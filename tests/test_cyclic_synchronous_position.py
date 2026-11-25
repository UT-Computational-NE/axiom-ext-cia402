# Copyright (c) 2026 The University of Texas at Austin
# Copyright (c) 2026 B-Tree Labs
# SPDX-License-Identifier: Apache-2.0
"""Cyclic synchronous position: the host sends a target every cycle and the drive follows."""

from __future__ import annotations

import math

import pytest

from axiom_ext_cia402 import objects as o


def enabled(parts):
    client, transport, drive = parts
    client.enable()
    assert client.run(0.01).state == o.OPERATION_ENABLED
    return client, transport, drive


def test_the_drive_says_it_follows_the_command(csp):
    client, _, drive = enabled(csp)
    assert drive.od.read(o.STATUSWORD) & o.SW_BIT_12
    assert client.status.mode == "cyclic synchronous position"
    assert drive.od.read(o.INTERPOLATION_TIME_PERIOD, 1) == 1  # the bus cycle, 1 ms


def test_a_streamed_sine_is_tracked_within_the_loops_lag(csp):
    client, transport, drive = enabled(csp)
    worst = 0.0
    for n in range(4000):  # 4 s of a 2 mm, 0.5 Hz sine
        t = n * transport.cycle_s
        want = 150.0 + 2.0 * math.sin(2 * math.pi * 0.5 * t)
        client.move_to(want)
        status = client.cycle()
        if n > 500:
            worst = max(worst, abs(status.position - want))
    # peak speed 2*pi*0.5*2 = 6.3 mm/s, times the 20 ms time constant
    assert worst == pytest.approx(6.283 * drive.axis.servo_time_constant_s, rel=0.15)
    assert not client.status.following_error_flag


def test_a_step_larger_than_the_drive_can_travel_in_a_cycle_is_rate_limited(csp):
    client, _, _ = enabled(csp)
    client.move_to(160.0)  # a 10 mm jump in one cycle; the drive allows 30 mm/s
    client.cycle()
    assert client.status.internal_limit
    assert client.run(0.2).position < 157.0  # about 30 mm/s, not instant
    assert client.run(1.0).position == pytest.approx(160.0, abs=0.01)


def test_a_host_updating_at_10_hz_gives_a_staircase_the_drive_smooths(csp):
    """A 10 Hz control loop above a 1 ms bus: the target changes every 100 cycles."""
    client, _, _ = enabled(csp)
    target, positions = 150.0, []
    for _ in range(20):
        target += 0.1  # 1 mm/s, in 0.1 mm steps
        client.move_to(target)
        positions.append(client.run(0.1).position)
    assert positions[-1] == pytest.approx(152.0, abs=0.01)
    assert all(b > a for a, b in zip(positions, positions[1:], strict=False))
