# Copyright (c) 2026 The University of Texas at Austin
# SPDX-License-Identifier: Apache-2.0
"""Objects are typed, and a bad access is refused with the abort code a drive returns."""

from __future__ import annotations

import pytest

from axiom_ext_cia402 import EmulatedDrive, SdoAbort
from axiom_ext_cia402 import dictionary as d
from axiom_ext_cia402 import objects as o


def abort(call) -> int:
    with pytest.raises(SdoAbort) as refused:
        call()
    return refused.value.code


def test_a_read_only_object_cannot_be_written():
    od = EmulatedDrive().od
    assert abort(lambda: od.write(o.STATUSWORD, 0, 0)) == d.ABORT_READ_ONLY
    assert abort(lambda: od.write(o.POSITION_ACTUAL_VALUE, 0, 5)) == d.ABORT_READ_ONLY


def test_an_unknown_object_or_subindex_is_named():
    od = EmulatedDrive().od
    assert abort(lambda: od.read(0x2000)) == d.ABORT_NO_OBJECT
    assert abort(lambda: od.read(o.SOFTWARE_POSITION_LIMIT, 3)) == d.ABORT_NO_SUBINDEX


def test_a_value_outside_the_objects_type_is_refused():
    od = EmulatedDrive().od
    assert abort(lambda: od.write(o.TARGET_POSITION, 0, 2**31)) == d.ABORT_OUT_OF_RANGE
    assert abort(lambda: od.write(o.PROFILE_VELOCITY, 0, -1)) == d.ABORT_OUT_OF_RANGE
    assert abort(lambda: od.write(o.MODES_OF_OPERATION, 0, 1.5)) == d.ABORT_OUT_OF_RANGE
    od.write(o.TARGET_POSITION, 0, -(2**31))


def test_the_drive_states_which_modes_it_supports():
    supported = EmulatedDrive().od.read(o.SUPPORTED_DRIVE_MODES)
    assert supported >> 0 & 1 and supported >> 5 & 1 and supported >> 7 & 1
    assert not supported >> 2 & 1  # profile velocity is not offered
