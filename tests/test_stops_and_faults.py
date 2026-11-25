# Copyright (c) 2026 The University of Texas at Austin
# SPDX-License-Identifier: Apache-2.0
"""Quick stop, a stalled axis, a lost bus, and getting back from a fault."""

from __future__ import annotations

import pytest
from conftest import make

from axiom_ext_cia402 import objects as o


def moving(mode="profile position", **overrides):
    client, transport, drive = make(mode, **overrides)
    client.enable()
    client.run(0.01)
    client.move_to(170.0)
    client.run(0.5)  # at the profile velocity
    return client, transport, drive


def test_quick_stop_decelerates_on_its_own_ramp_and_ends_switch_on_disabled():
    client, _, _ = moving()
    before = client.status.position
    client.quick_stop()
    during = client.run(0.02)
    assert during.state == o.QUICK_STOP_ACTIVE and during.velocity > 0  # still slowing
    after = client.run(0.5)
    assert after.state == o.SWITCH_ON_DISABLED and after.velocity == 0.0
    # from 10 mm/s at 200 mm/s^2 the profile stops in 0.25 mm; the axis was 0.2 mm behind it
    assert after.position - before == pytest.approx(0.25 + 0.2, abs=0.08)


def test_with_option_code_6_the_drive_stays_in_quick_stop_and_can_be_enabled_again():
    client, _, _ = moving(quick_stop_option_code=6)
    client.quick_stop()
    held = client.run(0.5)
    assert held.state == o.QUICK_STOP_ACTIVE and held.target_reached
    client.enable()
    assert client.run(0.01).state == o.OPERATION_ENABLED
    assert client.run(0.5).position == pytest.approx(held.position, abs=0.01)  # no old target


def test_a_stalled_axis_faults_on_following_error_and_holds():
    client, _, drive = moving()
    drive.stalled = True
    stuck = client.status.position
    status = client.run(0.5)
    assert status.state == o.FAULT
    assert (status.error_code, status.error) == (o.ERR_FOLLOWING, "following error")
    assert status.position == pytest.approx(stuck, abs=0.01)


def test_a_fault_needs_a_reset_and_then_a_fresh_enable():
    client, _, drive = moving()
    drive.stalled = True
    client.run(0.5)
    drive.stalled = False
    client.enable()
    assert client.run(0.05).state == o.FAULT  # enabling alone does not clear it
    client.fault_reset()
    cleared = client.run(0.01)
    assert cleared.state == o.SWITCH_ON_DISABLED and cleared.error_code == o.ERR_NONE
    client.enable()
    assert client.run(0.01).state == o.OPERATION_ENABLED
    client.move_to(cleared.position + 1.0)
    assert client.run(1.0).target_reached


def test_a_lost_bus_faults_the_drive_after_its_watchdog_time_and_it_stops():
    client, transport, drive = moving("cyclic synchronous position")
    transport.connected = False
    with pytest.raises(ConnectionError):
        client.cycle()
    transport.idle(0.05)
    assert drive.state == o.OPERATION_ENABLED  # inside the 100 ms watchdog
    transport.idle(0.2)
    assert drive.state == o.FAULT and drive.od.read(o.ERROR_CODE) == o.ERR_COMMUNICATION
    stopped = drive.od.read(o.POSITION_ACTUAL_VALUE)
    transport.idle(1.0)
    assert drive.od.read(o.POSITION_ACTUAL_VALUE) == stopped
    transport.connected = True
    assert client.cycle().state == o.FAULT  # the host finds it in fault when the bus returns
    assert client.status.error == "communication lost (process-data watchdog)"


def test_a_host_that_stops_cycling_is_a_lost_bus_to_the_drive():
    client, transport, drive = moving()
    transport.idle(0.3)  # the host process stalled
    assert drive.state == o.FAULT and drive.od.read(o.ERROR_CODE) == o.ERR_COMMUNICATION


def test_a_run_is_repeatable_to_the_count():
    def run():
        client, _, drive = moving("cyclic synchronous position")
        return [drive.od.read(o.POSITION_ACTUAL_VALUE)] + [
            round(client.run(0.01).position * 1e6) for _ in range(50)
        ]

    assert run() == run()


def test_a_reset_asked_for_during_the_fault_reaction_is_held_until_the_drive_can_take_it():
    client, transport, drive = moving("cyclic synchronous position")
    client.move_to(180.0)
    client.run(0.05)  # racing at the drive's 30 mm/s limit
    drive.fault(o.ERR_FOLLOWING)
    client.fault_reset()
    assert client.cycle().state == o.FAULT_REACTION_ACTIVE  # still ramping down: reset ignored
    assert client.run(0.5).state == o.SWITCH_ON_DISABLED  # taken once the drive reached fault
    assert client.status.error_code == o.ERR_NONE


def test_a_reset_with_no_fault_does_nothing():
    client, _, _ = moving()
    client.enable()
    client.fault_reset()
    assert client.run(0.01).state == o.SWITCH_ON_DISABLED  # a reset also withdraws the enable
    client.enable()
    assert client.run(0.01).state == o.OPERATION_ENABLED
