# Copyright (c) 2026 The University of Texas at Austin
# SPDX-License-Identifier: Apache-2.0
"""``python -m axiom_ext_cia402``: run an emulated drive through its paces and print what it did."""

from __future__ import annotations

import sys

from . import Cia402Client, EmulatedDrive, EmulatedTransport, Scaling, __version__
from . import objects as o


def main() -> int:
    failed: list[str] = []

    def check(what: str, ok: bool) -> None:
        print(("ok    " if ok else "FAIL  ") + what)
        if not ok:
            failed.append(what)

    drive = EmulatedDrive(position=150_000)
    transport = EmulatedTransport(drive, cycle_s=0.001)
    client = Cia402Client(transport, Scaling(1000.0, "mm"))
    client.configure(
        mode="profile position", limits=(0.0, 305.0), max_velocity=30.0, profile_velocity=10.0,
        profile_acceleration=50.0, profile_deceleration=50.0, quick_stop_deceleration=200.0,
        following_error_window=1.0, position_window=0.01,
    )  # fmt: skip
    print(f"axiom-ext-cia402 {__version__}: an emulated drive at 150 mm, 1 ms bus cycle")
    client.enable()
    check("enable reaches operation enabled", client.run(0.01).state == o.OPERATION_ENABLED)
    client.move_to(155.0)
    status = client.run(2.0)
    check(
        "a 5 mm move ends on target", abs(status.position - 155.0) < 0.01 and status.target_reached
    )
    client.move_to(400.0)
    client.cycle()
    check("a target beyond the travel is held to the limit", client.status.internal_limit)
    client.run(0.3)
    client.quick_stop()
    check("quick stop ends switch on disabled", client.run(0.5).state == o.SWITCH_ON_DISABLED)
    client.enable()
    client.run(0.01)
    client.move_to(client.status.position + 5.0)
    client.run(0.3)
    drive.stalled = True
    check("a stalled axis faults on following error", client.run(0.5).error_code == o.ERR_FOLLOWING)
    drive.stalled = False
    client.fault_reset()
    check(
        "fault reset returns to switch on disabled", client.run(0.01).state == o.SWITCH_ON_DISABLED
    )
    client.enable()
    client.run(0.01)
    transport.idle(0.3)
    check("a silent bus faults the drive", drive.od.read(o.ERROR_CODE) == o.ERR_COMMUNICATION)
    print("PASS" if not failed else f"FAIL: {len(failed)} check(s)")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
