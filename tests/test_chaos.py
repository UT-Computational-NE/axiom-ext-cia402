# Copyright (c) 2026 The University of Texas at Austin
# SPDX-License-Identifier: Apache-2.0
"""Chaos: random and extreme use, checked against what must hold whatever the host does.

Every case is seeded, so a failure repeats. The invariants:

* the statusword always states the drive's power state;
* the axis moves only while the power stage is on;
* the actual position stays inside the software limits it started inside;
* the actual speed never exceeds the maximum profile velocity;
* a fault leaves only by a reset;
* no value is ever not a number, and every object stays inside its type;
* the client raises nothing but its own refusals and a lost connection, and once the
  faults are gone it can always get the drive enabled and a move finished.
"""

from __future__ import annotations

import math
import random

import pytest
from conftest import make

from axiom_ext_cia402 import (
    AxisModel,
    Cia402Client,
    ClientRefused,
    EmulatedDrive,
    EmulatedTransport,
    Scaling,
    SdoAbort,
)
from axiom_ext_cia402 import objects as o

POWERED = (o.OPERATION_ENABLED, o.QUICK_STOP_ACTIVE, o.FAULT_REACTION_ACTIVE)
LOW, HIGH, V_MAX = 0, 305_000, 30_000


def raw_drive(seed: int, **axis) -> tuple[EmulatedDrive, random.Random]:
    rng = random.Random(seed)
    d = EmulatedDrive(AxisModel(**axis) if axis else None, position=150_000)
    d.od.write(o.SOFTWARE_POSITION_LIMIT, 1, LOW)
    d.od.write(o.SOFTWARE_POSITION_LIMIT, 2, HIGH)
    d.od.write(o.MAX_PROFILE_VELOCITY, 0, V_MAX)
    return d, rng


def check(d: EmulatedDrive, before: int, state_before: str, dt: float) -> None:
    sw = d.od.read(o.STATUSWORD)
    assert o.state_of(sw) == d.state
    actual = d.od.read(o.POSITION_ACTUAL_VALUE)
    if (
        state_before not in POWERED
        and d.state not in POWERED
        and d.od.read(o.MODES_OF_OPERATION_DISPLAY) != o.HOMING
    ):
        assert actual == before, "the axis moved with the power stage off"
    assert LOW - 1 <= actual <= HIGH + 1 or d.homed
    assert abs(d.od.read(o.VELOCITY_ACTUAL_VALUE)) <= V_MAX * 1.001 + 2 / dt or d.homed
    for (index, sub), entry in d.od.items():
        assert isinstance(entry.value, int), (hex(index), sub)
        assert entry.type.low <= entry.value <= entry.type.high, (hex(index), sub)
    assert math.isfinite(d._demand) and math.isfinite(d._actual)


@pytest.mark.parametrize("seed", range(40))
def test_random_controlwords_modes_and_targets_never_break_the_drive(seed):
    d, rng = raw_drive(seed)
    dt = rng.choice([0.0005, 0.001, 0.004])
    for _ in range(3000):
        if rng.random() < 0.3:
            words = [0x00, 0x02, 0x06, 0x07, 0x0F, 0x1F, 0x3F, 0x5F, 0x80, 0x8F, 0x10F]
            d.od.write(o.CONTROLWORD, 0, rng.choice([*words, rng.getrandbits(16)]))
        if rng.random() < 0.05:
            d.od.write(o.MODES_OF_OPERATION, 0, rng.choice([0, 1, 6, 8, 3, -1, 127]))
        if rng.random() < 0.2:
            d.od.write(o.TARGET_POSITION, 0, rng.choice(
                [rng.randint(LOW, HIGH), rng.randint(o.I32.low, o.I32.high), LOW, HIGH]
            ))  # fmt: skip
        if rng.random() < 0.01:
            d.stalled = not d.stalled
        if rng.random() < 0.9:
            d.exchanged()
        before, state_before = d.od.read(o.POSITION_ACTUAL_VALUE), d.state
        d.step(dt)
        check(d, before, state_before, dt)


@pytest.mark.parametrize("seed", range(20))
def test_a_fault_is_left_only_by_a_rising_edge_of_fault_reset(seed):
    d, rng = raw_drive(seed)
    for cw in (0x06, 0x07, 0x0F):
        d.od.write(o.CONTROLWORD, 0, cw)
        d.step(0.001)
    d.fault(o.ERR_FOLLOWING)
    for _ in range(500):
        cw = rng.getrandbits(16) & ~o.CW_FAULT_RESET
        d.od.write(o.CONTROLWORD, 0, cw)
        d.step(0.001)
        assert d.state == o.FAULT


@pytest.mark.parametrize(
    "index, value",
    [
        (o.PROFILE_VELOCITY, 0),
        (o.PROFILE_ACCELERATION, 0),
        (o.PROFILE_DECELERATION, 0),
        (o.QUICK_STOP_DECELERATION, 0),
        (o.MAX_PROFILE_VELOCITY, 0),
        (o.FOLLOWING_ERROR_WINDOW, 0),
        (o.FOLLOWING_ERROR_WINDOW, o.U32.high),
        (o.POSITION_WINDOW, o.U32.high),
        (o.PROFILE_VELOCITY, o.U32.high),
        (o.PROFILE_ACCELERATION, o.U32.high),
    ],
)
@pytest.mark.parametrize("mode", [o.PROFILE_POSITION, o.CYCLIC_SYNCHRONOUS_POSITION])
def test_an_extreme_setting_never_hangs_the_drive_or_gives_a_bad_number(index, value, mode):
    d, _ = raw_drive(0)
    d.od.write(index, 0, value)
    d.od.write(o.MODES_OF_OPERATION, 0, mode)
    script = [0x06, 0x07, 0x0F] + [0x1F] * 5 + [0x0F] * 400 + [0x02] * 400 + [0x00] * 5
    d.od.write(o.TARGET_POSITION, 0, 200_000)
    for cw in script:
        d.od.write(o.CONTROLWORD, 0, cw)
        d.exchanged()
        before, state_before = d.od.read(o.POSITION_ACTUAL_VALUE), d.state
        d.step(0.001)
        check(d, before, state_before, 0.001)
    # A quick stop always ends, whatever the deceleration: the drive is not left decelerating.
    assert d.state in (o.SWITCH_ON_DISABLED, o.FAULT)


@pytest.mark.parametrize("tau", [0.0, 1e-6, 0.02, 5.0])
@pytest.mark.parametrize("cycle", [1e-5, 0.001, 0.1])
def test_any_loop_time_constant_and_bus_cycle_gives_finite_bounded_motion(tau, cycle):
    scaling = Scaling(1000.0)
    drive = EmulatedDrive(AxisModel(servo_time_constant_s=tau, watchdog_s=0.0), position=150_000)
    client = Cia402Client(EmulatedTransport(drive, cycle_s=cycle), scaling)
    client.configure(mode="profile position", limits=(0.0, 305.0), following_error_window=1e6)
    client.enable()
    client.run(5 * cycle)
    client.move_to(151.0)
    for _ in range(300):
        status = client.cycle()
        assert math.isfinite(status.position) and 149.99 <= status.position <= 151.01


def test_inverted_limits_written_raw_do_not_trap_the_axis():
    d, _ = raw_drive(0)
    d.od.write(o.SOFTWARE_POSITION_LIMIT, 1, 200_000)
    d.od.write(o.SOFTWARE_POSITION_LIMIT, 2, 100_000)  # minimum above maximum
    d.od.write(o.MODES_OF_OPERATION, 0, o.PROFILE_POSITION)
    d.od.write(o.TARGET_POSITION, 0, 151_000)
    for cw in [0x06, 0x07, 0x0F, 0x1F] + [0x0F] * 2000:
        d.od.write(o.CONTROLWORD, 0, cw)
        d.exchanged()
        d.step(0.001)
    # Limits that contradict each other are treated as no limits, and the move is made.
    assert d.od.read(o.POSITION_ACTUAL_VALUE) == pytest.approx(151_000, abs=10)


@pytest.mark.parametrize("seed", range(30))
@pytest.mark.parametrize("mode", ["profile position", "cyclic synchronous position"])
def test_random_use_of_the_client_raises_only_its_own_refusals_and_always_recovers(seed, mode):
    rng = random.Random(seed)
    client, transport, drive = make(mode)
    for _ in range(400):
        act = rng.random()
        try:
            if act < 0.15:
                client.enable()
            elif act < 0.22:
                client.disable()
            elif act < 0.27:
                client.quick_stop()
            elif act < 0.33:
                client.fault_reset()
            elif act < 0.60:
                client.move_to(rng.uniform(-50.0, 400.0))
            elif act < 0.63:
                client.configure(mode=mode, profile_velocity=rng.uniform(0.0, 50.0))
            elif act < 0.66:
                drive.stalled = not drive.stalled
            elif act < 0.69:
                transport.connected = not transport.connected
            elif act < 0.71:
                transport.idle(rng.uniform(0.0, 0.3))
            client.run(rng.uniform(0.001, 0.05))
        except (ClientRefused, ConnectionError, SdoAbort):
            pass
        if client.status is not None:
            assert client.status.state in o.STATE_CODES
            assert -0.01 <= client.status.position <= 305.01

    # With the faults gone, the host can always get back to a finished move.
    drive.stalled, transport.connected = False, True
    client.disable()
    client.run(0.01)
    client.fault_reset()
    client.run(1.0)  # long enough for any fault reaction ramp to end and the reset to be taken
    assert client.status.state == o.SWITCH_ON_DISABLED
    client.configure(mode=mode, profile_velocity=10.0)
    client.enable()
    assert client.run(0.02).state == o.OPERATION_ENABLED
    client.move_to(100.0)
    done = client.run(40.0)
    assert done.position == pytest.approx(100.0, abs=0.02) and done.state == o.OPERATION_ENABLED


def test_a_long_run_does_not_drift_and_stays_fast():
    import time

    client, transport, drive = make("cyclic synchronous position")
    client.enable()
    client.run(0.01)
    started = time.perf_counter()
    for n in range(300_000):  # five minutes of bus time at 1 ms
        client.move_to(150.0 + 20.0 * math.sin(2 * math.pi * 0.2 * n * 0.001))
        client.cycle()
    per_cycle_us = (time.perf_counter() - started) / 300_000 * 1e6
    assert drive.time_s == pytest.approx(300.01, abs=1e-6)  # simulated time does not drift
    want = 150.0 + 20.0 * math.sin(2 * math.pi * 0.2 * 299_999 * 0.001)
    assert client.status.position == pytest.approx(want, abs=0.6)
    assert client.status.state == o.OPERATION_ENABLED
    assert per_cycle_us < 500, f"{per_cycle_us:.0f} us per cycle"
