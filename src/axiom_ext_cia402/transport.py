# Copyright (c) 2026 The University of Texas at Austin
# SPDX-License-Identifier: Apache-2.0
"""How a host reaches a drive's objects: the one seam between the client and the bus.

A transport does two things, as a fieldbus master does for a CoE drive:

* ``sdo_read`` and ``sdo_write``: one object at a time, for configuration;
* ``exchange``: one process-data cycle. The host's outputs go to the drive (the controlword
  and the targets), the drive's inputs come back (the statusword and the actual values),
  and one bus cycle of time passes.

:class:`EmulatedTransport` carries these to an :class:`~axiom_ext_cia402.drive.EmulatedDrive`
in the same process, on simulated time. A transport for a real bus implements the same three
methods over an EtherCAT master; this package does not contain one.
"""

from __future__ import annotations

from typing import Protocol

from . import objects as o
from .drive import EmulatedDrive

Address = tuple[int, int]  # (index, subindex)

#: What the host sends each cycle (the drive's RxPDO).
OUTPUTS: tuple[Address, ...] = (
    (o.CONTROLWORD, 0),
    (o.MODES_OF_OPERATION, 0),
    (o.TARGET_POSITION, 0),
)
#: What the drive sends back each cycle (its TxPDO).
INPUTS: tuple[Address, ...] = (
    (o.STATUSWORD, 0),
    (o.MODES_OF_OPERATION_DISPLAY, 0),
    (o.POSITION_ACTUAL_VALUE, 0),
    (o.VELOCITY_ACTUAL_VALUE, 0),
    (o.FOLLOWING_ERROR_ACTUAL_VALUE, 0),
    (o.ERROR_CODE, 0),
)


class Transport(Protocol):
    cycle_s: float

    def sdo_read(self, index: int, subindex: int = 0) -> int: ...

    def sdo_write(self, index: int, subindex: int, value: int) -> None: ...

    def exchange(self, outputs: dict[Address, int]) -> dict[Address, int]: ...


class EmulatedTransport:
    """An emulated drive on an emulated bus with a fixed cycle time."""

    def __init__(self, drive: EmulatedDrive, cycle_s: float = 0.001) -> None:
        if cycle_s <= 0:
            raise ValueError("the bus cycle has a positive duration")
        self.drive, self.cycle_s = drive, cycle_s
        self.connected = True  # False: the cable is out; cycles pass and nothing is exchanged
        self.cycles = 0

    def sdo_read(self, index: int, subindex: int = 0) -> int:
        self._require_link()
        return self.drive.od.read(index, subindex)

    def sdo_write(self, index: int, subindex: int, value: int) -> None:
        self._require_link()
        self.drive.od.write(index, subindex, value)

    def exchange(self, outputs: dict[Address, int]) -> dict[Address, int]:
        self.cycles += 1
        if not self.connected:
            self.drive.step(self.cycle_s)
            raise ConnectionError("no process data: the drive did not answer this cycle")
        for (index, subindex), value in outputs.items():
            self.drive.od.write(index, subindex, value)
        self.drive.exchanged()
        self.drive.step(self.cycle_s)
        return {address: self.drive.od.read(*address) for address in INPUTS}

    def idle(self, seconds: float) -> None:
        """Time passes with no host cycle at all (the host stalled or stopped)."""
        for _ in range(round(seconds / self.cycle_s)):
            self.drive.step(self.cycle_s)

    def _require_link(self) -> None:
        if not self.connected:
            raise ConnectionError("no answer from the drive")
