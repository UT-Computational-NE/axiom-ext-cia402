# Copyright (c) 2026 The University of Texas at Austin
# Copyright (c) 2026 B-Tree Labs
# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import pytest

from axiom_ext_cia402 import Cia402Client, EmulatedDrive, EmulatedTransport, Scaling

COUNTS_PER_MM = 1000.0
CYCLE_S = 0.001
ROD = dict(
    limits=(0.0, 305.0),
    max_velocity=30.0,  # mm/s
    profile_velocity=10.0,
    profile_acceleration=50.0,  # mm/s^2
    profile_deceleration=50.0,
    quick_stop_deceleration=200.0,
    following_error_window=1.0,  # mm
    position_window=0.01,
)


def make(mode="profile position", start_mm=150.0, **overrides):
    scaling = Scaling(COUNTS_PER_MM)
    drive = EmulatedDrive(position=scaling.to_counts(start_mm))
    transport = EmulatedTransport(drive, cycle_s=CYCLE_S)
    client = Cia402Client(transport, scaling)
    client.configure(mode=mode, **{**ROD, **overrides})
    return client, transport, drive


@pytest.fixture
def pp():
    return make("profile position")


@pytest.fixture
def csp():
    return make("cyclic synchronous position")
