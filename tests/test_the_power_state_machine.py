# Copyright (c) 2026 The University of Texas at Austin
# Copyright (c) 2026 B-Tree Labs
# SPDX-License-Identifier: Apache-2.0
"""The drive's power states and the controlword commands that move between them."""

from __future__ import annotations

import pytest

from axiom_ext_cia402 import EmulatedDrive
from axiom_ext_cia402 import objects as o


def drive_at(*commands):
    d = EmulatedDrive()
    d.od.write(o.MODES_OF_OPERATION, 0, o.PROFILE_POSITION)
    for cw in commands:
        d.od.write(o.CONTROLWORD, 0, cw)
        d.exchanged()
        d.step(0.001)
    return d


ENABLE = (o.CMD_SHUTDOWN, o.CMD_SWITCH_ON, o.CMD_ENABLE_OPERATION)


def test_a_drive_starts_switch_on_disabled_and_says_so_in_its_statusword():
    d = EmulatedDrive()
    assert d.state == o.SWITCH_ON_DISABLED
    assert o.state_of(d.od.read(o.STATUSWORD)) == o.SWITCH_ON_DISABLED
    assert d.od.read(o.DEVICE_TYPE) & 0xFFFF == 402


@pytest.mark.parametrize(
    "commands, state",
    [
        ((o.CMD_SHUTDOWN,), o.READY_TO_SWITCH_ON),  # transition 2
        ((o.CMD_SHUTDOWN, o.CMD_SWITCH_ON), o.SWITCHED_ON),  # 3
        (ENABLE, o.OPERATION_ENABLED),  # 4
        ((*ENABLE, o.CMD_SWITCH_ON), o.SWITCHED_ON),  # 5, disable operation
        ((o.CMD_SHUTDOWN, o.CMD_SWITCH_ON, o.CMD_SHUTDOWN), o.READY_TO_SWITCH_ON),  # 6
        ((o.CMD_SHUTDOWN, o.CMD_DISABLE_VOLTAGE), o.SWITCH_ON_DISABLED),  # 7
        ((*ENABLE, o.CMD_SHUTDOWN), o.READY_TO_SWITCH_ON),  # 8
        ((*ENABLE, o.CMD_DISABLE_VOLTAGE), o.SWITCH_ON_DISABLED),  # 9
        ((o.CMD_SHUTDOWN, o.CMD_SWITCH_ON, o.CMD_QUICK_STOP), o.SWITCH_ON_DISABLED),  # 10
    ],
)
def test_each_command_causes_its_transition(commands, state):
    d = drive_at(*commands)
    assert d.state == state
    assert o.state_of(d.od.read(o.STATUSWORD)) == state


def test_enable_operation_is_not_taken_from_switch_on_disabled():
    """The drive does not skip states: 0x0F from switch on disabled does nothing."""
    assert drive_at(o.CMD_ENABLE_OPERATION).state == o.SWITCH_ON_DISABLED


def test_enable_operation_from_ready_takes_two_cycles():
    assert drive_at(o.CMD_SHUTDOWN, o.CMD_ENABLE_OPERATION).state == o.SWITCHED_ON
    assert drive_at(o.CMD_SHUTDOWN, o.CMD_ENABLE_OPERATION, o.CMD_ENABLE_OPERATION).state == (
        o.OPERATION_ENABLED
    )


def test_a_fault_leaves_only_on_a_rising_edge_of_fault_reset():
    d = drive_at(*ENABLE)
    d.fault(o.ERR_FOLLOWING)
    assert d.state == o.FAULT and d.od.read(o.ERROR_CODE) == o.ERR_FOLLOWING
    for cw in (o.CMD_SHUTDOWN, o.CMD_ENABLE_OPERATION):  # ordinary commands do nothing
        d.od.write(o.CONTROLWORD, 0, cw)
        d.step(0.001)
        assert d.state == o.FAULT
    d.od.write(o.CONTROLWORD, 0, o.CW_FAULT_RESET)
    d.step(0.001)
    assert d.state == o.SWITCH_ON_DISABLED and d.od.read(o.ERROR_CODE) == o.ERR_NONE
    d.fault(o.ERR_FOLLOWING)
    d.step(0.001)  # bit 7 still high: no new edge, so no reset
    assert d.state == o.FAULT


def test_every_state_has_a_distinct_statusword_code():
    values = [value for _, value in o.STATE_CODES.values()]
    assert len(set(values)) == len(o.STATE_CODES)
    for name, (_, value) in o.STATE_CODES.items():
        assert o.state_of(value) == name
    with pytest.raises(ValueError):
        o.state_of(0x0001)
