# Copyright (c) 2026 The University of Texas at Austin
# SPDX-License-Identifier: Apache-2.0
"""An emulated CiA 402 servo drive: an object dictionary, the power state machine and motion.

The host sees what it would see from a real drive: it writes the controlword, the mode and
the target, and reads the statusword, the actual position and the rest. Time passes only
through :meth:`EmulatedDrive.step`, which the transport calls once per bus cycle, so a run
is repeatable to the bit.

What is modelled:

* the power state machine and every transition a controlword command causes;
* profile position: a trapezoidal profile from the profile velocity, acceleration and
  deceleration, the set-point handshake, one buffered set-point, change immediately,
  relative moves, halt, and target reached;
* cyclic synchronous position: the target is taken every cycle and interpolated;
* homing by methods 35 and 37 (the present position becomes the home position);
* the axis behind the profile: the actual position follows the demand through a first-order
  lag, so there is a following error, and it faults when the error stays outside its window;
* software position limits and the maximum profile velocity, with internal limit active;
* quick stop by option codes 1, 2, 5 and 6, the fault reaction, fault reset, error codes;
* a process-data watchdog: no cycle for longer than the watchdog time faults the drive.

What is not modelled: torque and current, the motor and the brake, velocity and torque
modes, the other homing methods, touch probes, and any manufacturer-specific object. A
real drive's values for these objects replace the defaults here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from . import objects as o
from .dictionary import ObjectDictionary


@dataclass(frozen=True)
class AxisModel:
    """What the emulator assumes about the axis. Every number is a placeholder until measured."""

    #: Time constant of the position loop: how the actual position lags the demand, seconds.
    servo_time_constant_s: float = 0.02
    #: No process data for this long, while the power stage is on, faults the drive. 0: off.
    watchdog_s: float = 0.1


class EmulatedDrive:
    def __init__(self, axis: AxisModel | None = None, *, position: int = 0) -> None:
        self.axis = axis or AxisModel()
        self.od = od = ObjectDictionary()
        ro = {"writable": False}
        od.add(o.DEVICE_TYPE, "device type", o.U32, 0x00020192, **ro)
        od.add(o.ERROR_CODE, "error code", o.U16, 0, **ro)
        od.add(o.CONTROLWORD, "controlword", o.U16, 0)
        od.add(o.STATUSWORD, "statusword", o.U16, 0, **ro)
        od.add(o.QUICK_STOP_OPTION_CODE, "quick stop option code", o.I16, 2)
        od.add(o.FAULT_REACTION_OPTION_CODE, "fault reaction option code", o.I16, 2)
        od.add(o.MODES_OF_OPERATION, "modes of operation", o.I8, o.NO_MODE)
        od.add(o.MODES_OF_OPERATION_DISPLAY, "modes of operation display", o.I8, o.NO_MODE, **ro)
        od.add(o.POSITION_DEMAND_VALUE, "position demand value", o.I32, position, **ro)
        od.add(o.POSITION_ACTUAL_VALUE, "position actual value", o.I32, position, **ro)
        od.add(o.FOLLOWING_ERROR_WINDOW, "following error window", o.U32, 2000)
        od.add(o.FOLLOWING_ERROR_TIME_OUT, "following error time out", o.U16, 50)  # ms
        od.add(o.POSITION_WINDOW, "position window", o.U32, 10)
        od.add(o.POSITION_WINDOW_TIME, "position window time", o.U16, 0)  # ms
        od.add(o.VELOCITY_ACTUAL_VALUE, "velocity actual value", o.I32, 0, **ro)
        od.add(o.TARGET_POSITION, "target position", o.I32, position)
        od.add(o.HOME_OFFSET, "home offset", o.I32, 0)
        od.add(o.SOFTWARE_POSITION_LIMIT, "min position limit", o.I32, o.I32.low, subindex=1)
        od.add(o.SOFTWARE_POSITION_LIMIT, "max position limit", o.I32, o.I32.high, subindex=2)
        od.add(o.MAX_PROFILE_VELOCITY, "max profile velocity", o.U32, 100_000)
        od.add(o.PROFILE_VELOCITY, "profile velocity", o.U32, 10_000)
        od.add(o.PROFILE_ACCELERATION, "profile acceleration", o.U32, 50_000)
        od.add(o.PROFILE_DECELERATION, "profile deceleration", o.U32, 50_000)
        od.add(o.QUICK_STOP_DECELERATION, "quick stop deceleration", o.U32, 200_000)
        od.add(o.HOMING_METHOD, "homing method", o.I8, 37)
        od.add(o.INTERPOLATION_TIME_PERIOD, "interpolation time value", o.U8, 1, subindex=1)
        od.add(o.INTERPOLATION_TIME_PERIOD, "interpolation time index", o.I8, -3, subindex=2)
        od.add(o.FOLLOWING_ERROR_ACTUAL_VALUE, "following error actual value", o.I32, 0, **ro)
        od.add(
            o.SUPPORTED_DRIVE_MODES, "supported drive modes", o.U32, o.SUPPORTED_MODES_BITS, **ro
        )

        self.state = o.SWITCH_ON_DISABLED  # a drive passes "not ready to switch on" by itself
        self.time_s = 0.0
        self._demand = float(position)  # where the profile says the axis should be
        self._demand_v = 0.0
        self._actual = float(position)
        self._actual_v = 0.0
        self._goal: float | None = None  # profile position: the set-point being travelled to
        self._buffered: float | None = None  # profile position: the one waiting behind it
        self._csp_goal = float(position)
        self._acknowledge = False
        self._previous_controlword = 0
        self._within_window_s = 0.0
        self._outside_error_s = 0.0
        self._since_exchange_s = 0.0
        self._stop_decel: float | None = None  # a quick stop or fault ramp in progress
        self._homed = False
        self._homing_error = False
        self._internal_limit = False
        # Faults a test can cause, as a bench would with a clamp or a pulled cable.
        self.stalled = False
        self._refresh()

    # ---- what the transport calls ------------------------------------------------------------

    def exchanged(self) -> None:
        """Process data arrived this cycle (feeds the watchdog)."""
        self._since_exchange_s = 0.0

    def step(self, dt: float) -> None:
        """One bus cycle of ``dt`` seconds: act on the controlword, then move."""
        if dt <= 0:
            raise ValueError("a cycle has a positive duration")
        self.time_s += dt
        self._since_exchange_s += dt
        self._power_state_machine()
        self.od.set(o.MODES_OF_OPERATION_DISPLAY, self._mode())
        self._move(dt)
        self._supervise(dt)
        self._refresh()

    # ---- the power state machine -------------------------------------------------------------

    def _power_state_machine(self) -> None:
        cw = self.od.read(o.CONTROLWORD)
        rising_reset = (
            bool(cw & o.CW_FAULT_RESET) and not self._previous_controlword & o.CW_FAULT_RESET
        )
        self._previous_controlword = cw
        state = self.state

        if state == o.FAULT:
            if rising_reset:  # transition 15
                self.od.set(o.ERROR_CODE, o.ERR_NONE)
                self._outside_error_s = 0.0
                self._since_exchange_s = 0.0
                self.state = o.SWITCH_ON_DISABLED
            return
        if state == o.FAULT_REACTION_ACTIVE:
            return  # leaves by itself when the reaction ramp ends (transition 14)

        disable_voltage = cw & 0x82 == 0x00
        quick_stop = cw & 0x86 == 0x02
        shutdown = cw & 0x87 == 0x06
        switch_on = cw & 0x8F == 0x07  # also "disable operation"
        enable = cw & 0x8F == 0x0F

        if state == o.SWITCH_ON_DISABLED:
            if shutdown:  # 2
                self.state = o.READY_TO_SWITCH_ON
        elif state == o.READY_TO_SWITCH_ON:
            if disable_voltage or quick_stop:  # 7
                self.state = o.SWITCH_ON_DISABLED
            elif switch_on or enable:  # 3 (enable is taken as switch on, then enable next cycle)
                self.state = o.SWITCHED_ON
        elif state == o.SWITCHED_ON:
            if disable_voltage or quick_stop:  # 10
                self.state = o.SWITCH_ON_DISABLED
            elif shutdown:  # 6
                self.state = o.READY_TO_SWITCH_ON
            elif enable:  # 4
                self._enter_operation()
        elif state == o.OPERATION_ENABLED:
            if disable_voltage:  # 9
                self._power_off(o.SWITCH_ON_DISABLED)
            elif quick_stop:  # 11
                self._begin_quick_stop()
            elif shutdown:  # 8
                self._power_off(o.READY_TO_SWITCH_ON)
            elif switch_on:  # 5
                self._power_off(o.SWITCHED_ON)
        elif state == o.QUICK_STOP_ACTIVE:
            if disable_voltage:  # 12
                self._power_off(o.SWITCH_ON_DISABLED)
            elif enable and self._stop_decel is None and self._stays_in_quick_stop():  # 16
                self._enter_operation()

    def _enter_operation(self) -> None:
        self.state = o.OPERATION_ENABLED
        self._demand, self._demand_v = self._actual, 0.0
        self._goal = self._buffered = None
        self._csp_goal = self._actual
        self._stop_decel = None

    def _power_off(self, state: str) -> None:
        """The power stage goes off: the axis is held where it is (a brake is assumed)."""
        self.state = state
        self._demand, self._demand_v = self._actual, 0.0
        self._actual_v = 0.0
        self._goal = self._buffered = None
        self._stop_decel = None

    def _stays_in_quick_stop(self) -> bool:
        return self.od.read(o.QUICK_STOP_OPTION_CODE) in (5, 6)

    def _begin_quick_stop(self) -> None:
        code = self.od.read(o.QUICK_STOP_OPTION_CODE)
        self.state = o.QUICK_STOP_ACTIVE
        self._goal = self._buffered = None
        if code == 0:  # disable the drive function
            self._power_off(o.SWITCH_ON_DISABLED)
            return
        ramp = o.PROFILE_DECELERATION if code in (1, 5) else o.QUICK_STOP_DECELERATION
        self._stop_decel = float(self.od.read(ramp))

    def fault(self, error_code: int) -> None:
        """Enter the fault reaction with ``error_code`` (transition 13)."""
        if self.state in (o.FAULT, o.FAULT_REACTION_ACTIVE):
            return
        self.od.set(o.ERROR_CODE, error_code)
        powered = self.state in (o.OPERATION_ENABLED, o.QUICK_STOP_ACTIVE)
        self._goal = self._buffered = None
        if powered and self.od.read(o.FAULT_REACTION_OPTION_CODE) != 0 and self._demand_v:
            self.state = o.FAULT_REACTION_ACTIVE
            self._stop_decel = float(self.od.read(o.QUICK_STOP_DECELERATION))
        else:
            self._power_off(o.FAULT)

    # ---- motion ------------------------------------------------------------------------------

    def _mode(self) -> int:
        mode = self.od.read(o.MODES_OF_OPERATION)
        return mode if mode in o.MODE_NAMES else o.NO_MODE

    def _limits(self) -> tuple[int, int]:
        return (
            self.od.read(o.SOFTWARE_POSITION_LIMIT, 1),
            self.od.read(o.SOFTWARE_POSITION_LIMIT, 2),
        )

    def _held_to_limits(self, position: float) -> float:
        low, high = self._limits()
        held = min(max(position, low), high)
        if held != position:
            self._internal_limit = True
        return held

    def _move(self, dt: float) -> None:
        self._internal_limit = False
        if self._stop_decel is not None:
            self._ramp_down(dt)
        elif self.state == o.OPERATION_ENABLED:
            mode, cw = self._mode(), self.od.read(o.CONTROLWORD)
            if mode == o.PROFILE_POSITION:
                self._profile_position(cw, dt)
            elif mode == o.CYCLIC_SYNCHRONOUS_POSITION:
                self._cyclic_position(dt)
            elif mode == o.HOMING:
                self._homing(cw)
        # The axis: the actual position follows the demand through the position loop's lag.
        before = self._actual
        if self.stalled:
            pass
        elif self.state in (o.OPERATION_ENABLED, o.QUICK_STOP_ACTIVE, o.FAULT_REACTION_ACTIVE):
            alpha = 1.0 - math.exp(-dt / self.axis.servo_time_constant_s)
            self._actual += alpha * (self._demand - self._actual)
        self._actual_v = (self._actual - before) / dt

    def _ramp_down(self, dt: float) -> None:
        speed = abs(self._demand_v)
        drop = self._stop_decel * dt
        if speed <= drop:
            self._demand += (
                self._demand_v * (speed / self._stop_decel if self._stop_decel else 0) / 2
            )
            self._demand_v = 0.0
            self._stop_decel = None
            if self.state == o.FAULT_REACTION_ACTIVE:
                self._power_off(o.FAULT)  # 14
            elif self.state == o.QUICK_STOP_ACTIVE and not self._stays_in_quick_stop():
                self._power_off(o.SWITCH_ON_DISABLED)  # 12
            return
        new_v = math.copysign(speed - drop, self._demand_v)
        self._demand += (self._demand_v + new_v) / 2 * dt
        self._demand_v = new_v

    def _profile_position(self, cw: int, dt: float) -> None:
        new = bool(cw & o.CW_NEW_SET_POINT)
        if new and not self._acknowledge:
            target = float(self.od.read(o.TARGET_POSITION))
            if cw & o.CW_RELATIVE:
                target += self._goal if self._goal is not None else self._demand
            target = self._held_to_limits(target)
            if self._goal is None or cw & o.CW_CHANGE_SET_IMMEDIATELY:
                self._goal, self._buffered = target, None
            else:
                self._buffered = target
            self._acknowledge = True
        elif not new and self._acknowledge:
            self._acknowledge = False

        if self._goal is None:
            self._demand_v = 0.0
            return
        v_max = float(min(self.od.read(o.PROFILE_VELOCITY), self.od.read(o.MAX_PROFILE_VELOCITY)))
        accel = float(self.od.read(o.PROFILE_ACCELERATION))
        decel = float(self.od.read(o.PROFILE_DECELERATION))
        goal = self._demand if cw & o.CW_HALT else self._goal
        self._trapezoid(goal, 0.0 if cw & o.CW_HALT else v_max, accel, decel, dt)
        if self._demand == self._goal and self._demand_v == 0.0 and self._buffered is not None:
            self._goal, self._buffered = self._buffered, None

    def _trapezoid(self, goal: float, v_max: float, accel: float, decel: float, dt: float) -> None:
        """Advance the demand toward ``goal`` under a velocity and acceleration limit."""
        gap = goal - self._demand
        v = self._demand_v
        if v_max == 0.0 or (gap == 0.0 and v == 0.0):  # halt, or already there
            if v:
                new_v = math.copysign(max(abs(v) - decel * dt, 0.0), v)
                self._demand += (v + new_v) / 2 * dt
                self._demand_v = new_v
            return
        direction = math.copysign(1.0, gap) if gap else math.copysign(1.0, v)
        along = v * direction  # speed toward the goal; negative when moving away
        if along < 0:  # moving the wrong way: brake first
            new_along = min(along + decel * dt, 0.0)
        else:
            # the fastest speed from which the remaining distance still allows a stop
            can_stop_from = math.sqrt(2.0 * decel * abs(gap)) if decel else v_max
            new_along = min(along + accel * dt, v_max, can_stop_from)
        travel = (along + new_along) / 2 * dt
        if new_along >= 0 and travel >= abs(gap):
            self._demand, self._demand_v = goal, 0.0
            return
        self._demand += travel * direction
        self._demand_v = new_along * direction

    def _cyclic_position(self, dt: float) -> None:
        self._csp_goal = self._held_to_limits(float(self.od.read(o.TARGET_POSITION)))
        v_max = float(self.od.read(o.MAX_PROFILE_VELOCITY))
        gap = self._csp_goal - self._demand
        reach = v_max * dt
        if abs(gap) > reach:  # asked to go faster than the drive allows
            self._internal_limit = True
            gap = math.copysign(reach, gap)
        self._demand += gap
        self._demand_v = gap / dt

    def _homing(self, cw: int) -> None:
        self._demand_v = 0.0
        if not cw & o.CW_NEW_SET_POINT:  # bit 4 is "homing operation start" in this mode
            return
        if self.od.read(o.HOMING_METHOD) in (35, 37):
            home = float(self.od.read(o.HOME_OFFSET))
            self._actual = self._demand = self._csp_goal = home
            self._homed, self._homing_error = True, False
        else:
            self._homed, self._homing_error = False, True

    # ---- supervision and the statusword --------------------------------------------------------

    def _supervise(self, dt: float) -> None:
        powered = self.state in (o.OPERATION_ENABLED, o.QUICK_STOP_ACTIVE)
        error = self._demand - self._actual
        self.od.set(o.FOLLOWING_ERROR_ACTUAL_VALUE, round(error))
        outside = abs(error) > self.od.read(o.FOLLOWING_ERROR_WINDOW)
        self._outside_error_s = self._outside_error_s + dt if (powered and outside) else 0.0
        if powered and self._outside_error_s * 1000.0 > self.od.read(o.FOLLOWING_ERROR_TIME_OUT):
            self.fault(o.ERR_FOLLOWING)
        if powered and self.axis.watchdog_s and self._since_exchange_s > self.axis.watchdog_s:
            self.fault(o.ERR_COMMUNICATION)

        goal = self._goal if self._mode() == o.PROFILE_POSITION else self._csp_goal
        near = goal is not None and abs(goal - self._actual) <= self.od.read(o.POSITION_WINDOW)
        self._within_window_s = self._within_window_s + dt if near else 0.0

    def _refresh(self) -> None:
        od, mode = self.od, self._mode()
        od.set(o.POSITION_DEMAND_VALUE, round(self._demand))
        od.set(o.POSITION_ACTUAL_VALUE, round(self._actual))
        od.set(o.VELOCITY_ACTUAL_VALUE, round(self._actual_v))
        _, value = o.STATE_CODES[self.state]
        sw = value | o.SW_REMOTE
        if self.state in (o.SWITCHED_ON, o.OPERATION_ENABLED, o.QUICK_STOP_ACTIVE):
            sw |= o.SW_VOLTAGE_ENABLED
        if self.state == o.OPERATION_ENABLED:
            if self._internal_limit:
                sw |= o.SW_INTERNAL_LIMIT_ACTIVE
            reached = self._within_window_s * 1000.0 >= od.read(o.POSITION_WINDOW_TIME)
            if mode == o.PROFILE_POSITION:
                if self._goal is None or (reached and self._within_window_s > 0):
                    sw |= o.SW_TARGET_REACHED
                if self._acknowledge:
                    sw |= o.SW_BIT_12
            elif mode == o.CYCLIC_SYNCHRONOUS_POSITION:
                sw |= o.SW_BIT_12  # the drive follows the command value
            elif mode == o.HOMING:
                sw |= o.SW_TARGET_REACHED
                if self._homed:
                    sw |= o.SW_BIT_12
                if self._homing_error:
                    sw |= o.SW_BIT_13
            if mode != o.HOMING and abs(self._demand - self._actual) > od.read(
                o.FOLLOWING_ERROR_WINDOW
            ):
                sw |= o.SW_BIT_13
        elif self.state == o.QUICK_STOP_ACTIVE and self._stop_decel is None:
            sw |= o.SW_TARGET_REACHED  # the stop is complete
        od.set(o.STATUSWORD, sw)

    @property
    def homed(self) -> bool:
        return self._homed
