# Copyright (c) 2026 The University of Texas at Austin
# SPDX-License-Identifier: Apache-2.0
"""Profile position: the drive runs the trajectory, the host hands over set-points."""

from __future__ import annotations

import pytest

from cia402_emulator import ClientRefused
from cia402_emulator import objects as o


def enabled(parts):
    client, transport, drive = parts
    client.enable()
    assert client.run(0.01).state == o.OPERATION_ENABLED
    return client, transport, drive


def test_enabling_walks_the_state_machine_one_state_per_cycle(pp):
    client, _, _ = pp
    client.enable()
    states = [client.cycle().state for _ in range(4)]
    assert states == [o.READY_TO_SWITCH_ON, o.SWITCHED_ON, o.OPERATION_ENABLED, o.OPERATION_ENABLED]


def test_a_move_follows_a_trapezoid_and_ends_on_target(pp):
    client, _, drive = enabled(pp)
    client.move_to(155.0)
    speeds = []
    for _ in range(100):  # the first second, every 10 ms
        speeds.append(client.run(0.01).velocity)
    assert max(speeds) == pytest.approx(10.0, abs=0.05)  # held at the profile velocity
    assert speeds[5] < speeds[10] < speeds[19]  # ramping up at 50 mm/s^2 for 0.2 s
    status = client.run(1.0)
    assert status.position == pytest.approx(155.0, abs=0.01)
    assert status.target_reached and not status.following_error_flag


def test_target_reached_is_false_while_moving(pp):
    client, _, _ = enabled(pp)
    client.move_to(155.0)
    assert not client.run(0.2).target_reached


def test_the_set_point_handshake_raises_bit_4_until_acknowledged_then_lowers_it(pp):
    client, _, drive = enabled(pp)
    client.move_to(151.0)
    seen = []
    for _ in range(4):
        client.cycle()
        seen.append(
            (bool(drive.od.read(o.CONTROLWORD) & o.CW_NEW_SET_POINT),
             client.status.set_point_acknowledged)
        )  # fmt: skip
    assert seen == [(True, True), (False, False), (False, False), (False, False)]


def test_a_second_target_while_moving_replaces_the_first(pp):
    client, _, _ = enabled(pp)
    client.move_to(160.0)
    client.run(0.5)
    client.move_to(151.0)
    assert client.run(3.0).position == pytest.approx(151.0, abs=0.01)


def test_the_axis_lags_the_profile_by_the_speed_times_the_loops_time_constant(pp):
    client, _, drive = enabled(pp)
    client.move_to(160.0)
    lag = client.run(0.6).following_error
    assert lag == pytest.approx(10.0 * drive.axis.servo_time_constant_s, rel=0.05)


def test_a_target_beyond_the_travel_is_held_to_the_limit_and_the_drive_says_so(pp):
    client, _, drive = enabled(pp)
    drive.od.write(o.PROFILE_VELOCITY, 0, 30_000)
    client.move_to(400.0)
    client.cycle()
    assert client.status.internal_limit
    assert client.run(12.0).position == pytest.approx(305.0, abs=0.01)


def test_settings_cannot_be_changed_while_enabled(pp):
    client, _, _ = enabled(pp)
    with pytest.raises(ClientRefused, match="disable it"):
        client.configure(mode="profile position", profile_velocity=20.0)


def test_disabling_holds_the_axis_and_drops_the_target(pp):
    client, _, _ = enabled(pp)
    client.move_to(160.0)
    client.run(0.5)
    client.disable()
    held = client.run(0.01)
    assert held.state == o.SWITCH_ON_DISABLED and held.velocity == 0.0
    client.enable()
    again = client.run(1.0)
    assert again.state == o.OPERATION_ENABLED
    assert again.position == pytest.approx(held.position, abs=0.01)  # it did not resume the move


def test_homing_makes_the_present_position_the_home_position():
    from conftest import make

    client, _, drive = make("homing", start_mm=42.0)
    client.enable()
    client.run(0.01)
    assert not client.status.homed
    client.home_here()
    status = client.run(0.01)
    assert status.homed and status.position == 0.0 and drive.homed
