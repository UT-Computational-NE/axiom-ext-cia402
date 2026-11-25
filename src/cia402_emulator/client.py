# Copyright (c) 2026 The University of Texas at Austin
# SPDX-License-Identifier: Apache-2.0
"""The host side: drives a CiA 402 drive through a transport, in engineering units.

The client is the same whatever the transport reaches: the emulated drive or a real one.
It keeps what the host wants (enabled or not, a mode, a target) and, once per bus cycle in
:meth:`Cia402Client.cycle`, turns that into a controlword by following the drive's own state
machine: shutdown, switch on, enable operation, and the set-point handshake of profile
position. It never assumes a transition happened. It reads the statusword and acts on the
state the drive reports.

Positions are in the host's unit (``Scaling.unit``, for example mm). The drive's increments
appear only in :class:`Scaling`.
"""

from __future__ import annotations

from dataclasses import dataclass

from . import objects as o
from .transport import Transport


@dataclass(frozen=True)
class Scaling:
    """The one conversion between the host's unit and the drive's increments."""

    counts_per_unit: float
    unit: str = "mm"
    offset_counts: int = 0  # the drive's position at the host's zero

    def __post_init__(self) -> None:
        if self.counts_per_unit == 0:
            raise ValueError("counts_per_unit cannot be 0")

    def to_counts(self, value: float) -> int:
        return round(value * self.counts_per_unit) + self.offset_counts

    def to_unit(self, counts: int) -> float:
        return (counts - self.offset_counts) / self.counts_per_unit

    def rate_to_counts(self, value: float) -> int:
        return round(abs(value * self.counts_per_unit))

    def rate_to_unit(self, counts: int) -> float:
        return counts / self.counts_per_unit


@dataclass(frozen=True)
class DriveStatus:
    state: str
    mode: str
    position: float
    velocity: float
    following_error: float
    target_reached: bool
    internal_limit: bool
    following_error_flag: bool
    set_point_acknowledged: bool
    homed: bool
    error_code: int
    error: str
    statusword: int
    unit: str

    @property
    def enabled(self) -> bool:
        return self.state == o.OPERATION_ENABLED

    @property
    def faulted(self) -> bool:
        return self.state in (o.FAULT, o.FAULT_REACTION_ACTIVE)

    def record(self) -> dict:
        return {k: getattr(self, k) for k in self.__dataclass_fields__}


class ClientRefused(Exception):
    """A request the client will not make of the drive, with the reason in words."""


class Cia402Client:
    def __init__(self, transport: Transport, scaling: Scaling) -> None:
        self.transport, self.scaling = transport, scaling
        self._want_enabled = False
        self._want_quick_stop = False
        self._want_reset = False
        self._reset_low_sent = False  # the pulse on bit 7 starts low
        self._reset_fresh = False  # no status has been read since the reset was asked for
        self._mode = o.NO_MODE
        self._target_counts: int | None = None
        self._pending_set_point = False  # profile position: a set-point to hand over
        self._offered = False  # profile position: bit 4 is raised and not yet acknowledged
        self._start_homing = False
        self._homed = False
        self.status: DriveStatus | None = None

    # ---- configuration (SDO) -----------------------------------------------------------------

    def configure(
        self,
        *,
        mode: str,
        limits: tuple[float, float] | None = None,
        max_velocity: float | None = None,
        profile_velocity: float | None = None,
        profile_acceleration: float | None = None,
        profile_deceleration: float | None = None,
        quick_stop_deceleration: float | None = None,
        following_error_window: float | None = None,
        position_window: float | None = None,
        quick_stop_option_code: int | None = None,
    ) -> None:
        """Write the drive's settings, in the host's unit. Refused once the drive is enabled."""
        if self.status is not None and self.status.state == o.OPERATION_ENABLED:
            raise ClientRefused("the drive is enabled: disable it before changing its settings")
        if mode not in o.MODE_CODES or o.MODE_CODES[mode] == o.NO_MODE:
            raise ClientRefused(f"mode {mode!r} is not one of {sorted(o.MODE_CODES)[1:]}")
        supported = self.transport.sdo_read(o.SUPPORTED_DRIVE_MODES)
        bit = {o.PROFILE_POSITION: 0, o.HOMING: 5, o.CYCLIC_SYNCHRONOUS_POSITION: 7}[
            o.MODE_CODES[mode]
        ]
        if not supported >> bit & 1:
            raise ClientRefused(f"the drive does not support {mode} (0x6502 = {supported:#x})")
        self._mode = o.MODE_CODES[mode]
        write, s = self.transport.sdo_write, self.scaling
        if limits is not None:
            low, high = sorted(s.to_counts(x) for x in limits)
            write(o.SOFTWARE_POSITION_LIMIT, 1, low)
            write(o.SOFTWARE_POSITION_LIMIT, 2, high)
        for index, value in (
            (o.MAX_PROFILE_VELOCITY, max_velocity),
            (o.PROFILE_VELOCITY, profile_velocity),
            (o.PROFILE_ACCELERATION, profile_acceleration),
            (o.PROFILE_DECELERATION, profile_deceleration),
            (o.QUICK_STOP_DECELERATION, quick_stop_deceleration),
            (o.FOLLOWING_ERROR_WINDOW, following_error_window),
            (o.POSITION_WINDOW, position_window),
        ):
            if value is not None:
                write(index, 0, s.rate_to_counts(value))
        if quick_stop_option_code is not None:
            write(o.QUICK_STOP_OPTION_CODE, 0, quick_stop_option_code)
        if self._mode == o.CYCLIC_SYNCHRONOUS_POSITION:
            # the interpolation period is the bus cycle, in milliseconds
            write(o.INTERPOLATION_TIME_PERIOD, 1, max(1, round(self.transport.cycle_s * 1000)))
            write(o.INTERPOLATION_TIME_PERIOD, 2, -3)

    # ---- what the host asks for --------------------------------------------------------------

    def enable(self) -> None:
        if self._mode == o.NO_MODE:
            raise ClientRefused("no mode of operation is configured")
        self._want_enabled, self._want_quick_stop = True, False

    def disable(self) -> None:
        self._want_enabled, self._want_quick_stop = False, False
        self._pending_set_point = False

    def quick_stop(self) -> None:
        self._want_quick_stop = True
        self._pending_set_point = False

    def fault_reset(self) -> None:
        """Ask the drive to leave fault. It comes back disabled: enabling is a separate ask.

        A drive ignores a reset while its fault reaction is still under way, so the request
        is held until the drive reports fault, and then sent as a pulse."""
        self._want_reset, self._reset_low_sent, self._reset_fresh = True, False, True
        self._want_enabled = self._want_quick_stop = False

    def move_to(self, position: float) -> None:
        """A new target, in the host's unit. Profile position hands it over as a set-point;
        cyclic synchronous position sends it every cycle from now on."""
        if self._mode not in (o.PROFILE_POSITION, o.CYCLIC_SYNCHRONOUS_POSITION):
            raise ClientRefused("move_to needs profile position or cyclic synchronous position")
        self._target_counts = self.scaling.to_counts(position)
        self._pending_set_point = self._mode == o.PROFILE_POSITION

    def home_here(self) -> None:
        """Make the present position the home position (homing method 37)."""
        if self._mode != o.HOMING:
            raise ClientRefused("home_here needs the homing mode")
        self.transport.sdo_write(o.HOMING_METHOD, 0, 37)
        self._start_homing = True

    # ---- one bus cycle -----------------------------------------------------------------------

    def cycle(self) -> DriveStatus:
        """One process-data cycle: send what the state calls for, read the drive back."""
        state = self.status.state if self.status else None
        mode_bits = 0
        if (
            self._want_reset
            and not self._reset_fresh
            and state
            not in (
                o.FAULT,
                o.FAULT_REACTION_ACTIVE,
            )
        ):
            self._want_reset = False  # the drive, read since the request, is not in fault
        self._reset_fresh = False
        if self._want_reset:
            # a pulse on bit 7: low for one cycle, then high, so the drive sees a rising edge;
            # held low while the fault reaction runs, and repeated until the drive has left fault
            if state == o.FAULT and self._reset_low_sent:
                cw, self._reset_low_sent = o.CW_FAULT_RESET, False
            else:
                cw, self._reset_low_sent = 0, True
        elif self._want_quick_stop:
            cw = o.CMD_QUICK_STOP
        elif not self._want_enabled:
            cw = o.CMD_DISABLE_VOLTAGE
        elif state in (None, o.SWITCH_ON_DISABLED, o.NOT_READY_TO_SWITCH_ON):
            cw = o.CMD_SHUTDOWN
        elif state == o.READY_TO_SWITCH_ON:
            cw = o.CMD_SWITCH_ON
        elif state in (o.SWITCHED_ON, o.OPERATION_ENABLED):
            cw = o.CMD_ENABLE_OPERATION
            if state == o.OPERATION_ENABLED:
                mode_bits = self._mode_bits()
        elif state == o.QUICK_STOP_ACTIVE:
            cw = o.CMD_ENABLE_OPERATION  # taken only where the drive stays in quick stop
        else:  # fault: nothing but a reset moves it
            cw = o.CMD_DISABLE_VOLTAGE

        target = self._target_counts
        if target is None and self.status is not None:
            target = self.scaling.to_counts(self.status.position)
        outputs = {
            (o.CONTROLWORD, 0): cw | mode_bits,
            (o.MODES_OF_OPERATION, 0): self._mode,
        }
        if target is not None:
            outputs[(o.TARGET_POSITION, 0)] = target
        inputs = self.transport.exchange(outputs)
        self.status = self._read(inputs)
        if self.status.state != o.OPERATION_ENABLED:
            # A drive that has left operation has dropped its targets. Do not resend them.
            self._target_counts = None
            self._pending_set_point = self._offered = False
        return self.status

    def run(self, seconds: float) -> DriveStatus:
        """Cycle for ``seconds`` of bus time."""
        for _ in range(max(1, round(seconds / self.transport.cycle_s))):
            self.cycle()
        return self.status

    def _mode_bits(self) -> int:
        acknowledged = bool(self.status and self.status.set_point_acknowledged)
        if self._mode == o.PROFILE_POSITION:
            if self._offered:
                if acknowledged:  # the drive took it: lower bit 4
                    self._offered = self._pending_set_point = False
                    return 0
                return o.CW_NEW_SET_POINT | o.CW_CHANGE_SET_IMMEDIATELY
            if self._pending_set_point and not acknowledged:
                self._offered = True
                return o.CW_NEW_SET_POINT | o.CW_CHANGE_SET_IMMEDIATELY
        elif self._mode == o.HOMING and self._start_homing:
            if self.status and self.status.statusword & o.SW_BIT_12:
                self._start_homing, self._homed = False, True
                return 0
            return o.CW_NEW_SET_POINT
        return 0

    def _read(self, inputs: dict) -> DriveStatus:
        s = self.scaling
        sw = inputs[(o.STATUSWORD, 0)]
        mode = inputs[(o.MODES_OF_OPERATION_DISPLAY, 0)]
        code = inputs[(o.ERROR_CODE, 0)]
        if mode == o.HOMING and sw & o.SW_BIT_12:
            self._homed = True
        return DriveStatus(
            state=o.state_of(sw),
            mode=o.MODE_NAMES.get(mode, f"mode {mode}"),
            position=s.to_unit(inputs[(o.POSITION_ACTUAL_VALUE, 0)]),
            velocity=s.rate_to_unit(inputs[(o.VELOCITY_ACTUAL_VALUE, 0)]),
            following_error=s.rate_to_unit(inputs[(o.FOLLOWING_ERROR_ACTUAL_VALUE, 0)]),
            target_reached=bool(sw & o.SW_TARGET_REACHED),
            internal_limit=bool(sw & o.SW_INTERNAL_LIMIT_ACTIVE),
            following_error_flag=mode != o.HOMING and bool(sw & o.SW_BIT_13),
            set_point_acknowledged=mode == o.PROFILE_POSITION and bool(sw & o.SW_BIT_12),
            homed=self._homed,
            error_code=code,
            error=o.ERROR_NAMES.get(code, f"error {code:#06x}"),
            statusword=sw,
            unit=s.unit,
        )
