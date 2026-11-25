# Copyright (c) 2026 The University of Texas at Austin
# Copyright (c) 2026 B-Tree Labs
# SPDX-License-Identifier: Apache-2.0
"""What a record of a run needs from the client: which drive, what it was told, what happened
between reads, and how the bus behaved."""

from __future__ import annotations

import pytest
from conftest import make

from axiom_ext_cia402 import ClientRefused
from axiom_ext_cia402 import objects as o


def test_the_client_says_which_drive_it_is_talking_to(pp):
    client, _, _ = pp
    identity = client.identity()
    assert identity["device_type"] & 0xFFFF == 402
    assert set(identity) == {"device_type", "vendor_id", "product_code", "revision_number",
                             "serial_number"}  # fmt: skip


def test_every_setting_written_is_kept_with_what_the_drive_read_back(pp):
    client, _, _ = pp
    written = {(c["index"], c["subindex"]): c for c in client.configuration}
    assert written[(o.PROFILE_VELOCITY, 0)] == {
        "index": o.PROFILE_VELOCITY, "subindex": 0, "written": 10_000, "read_back": 10_000,
    }  # fmt: skip
    assert written[(o.SOFTWARE_POSITION_LIMIT, 2)]["read_back"] == 305_000
    assert all(c["written"] == c["read_back"] for c in client.configuration)


def test_a_setting_the_drive_does_not_take_is_refused(pp, monkeypatch):
    client, transport, _ = pp
    real = transport.sdo_read
    monkeypatch.setattr(
        transport, "sdo_read",
        lambda index, sub=0: 9_999 if index == o.PROFILE_VELOCITY else real(index, sub),
    )  # fmt: skip
    with pytest.raises(ClientRefused, match="0x6081:0 was written 10000 and reads back 9999"):
        client.configure(mode="profile position", profile_velocity=10.0)


def test_events_carry_every_state_the_drive_passed_through_with_its_bus_time(pp):
    client, _, _ = pp
    client.take_events()
    client.enable()
    client.run(0.01)
    events = client.take_events()
    assert events[0]["event"] == "asked" and events[0]["what"] == "enable"
    states = [e["now"] for e in events if e.get("what") == "state"]
    assert states == [o.READY_TO_SWITCH_ON, o.SWITCHED_ON, o.OPERATION_ENABLED]
    times = [e["bus_time_s"] for e in events]
    assert times == sorted(times) and times[-1] <= 0.01
    assert client.take_events() == []  # taken once


def test_a_fault_that_came_and_went_between_two_reads_is_still_in_the_events(pp):
    client, _, drive = pp
    client.enable()
    client.run(0.01)
    client.take_events()
    drive.fault(o.ERR_FOLLOWING)
    client.fault_reset()
    client.run(0.02)
    client.enable()
    status = client.run(0.02)
    assert status.state == o.OPERATION_ENABLED and status.error_code == o.ERR_NONE  # looks fine now
    events = client.take_events()
    fault = next(e for e in events if e.get("what") == "error_code" and e["now"] == o.ERR_FOLLOWING)
    assert fault["error"] == "following error" and "statusword" in fault
    assert [e["what"] for e in events if e["event"] == "asked"] == ["fault reset", "enable"]


def test_a_streamed_target_is_one_event_until_it_changes(csp):
    client, _, _ = csp
    client.enable()
    client.run(0.01)
    client.take_events()
    for _ in range(50):
        client.move_to(151.0)
        client.cycle()
    client.move_to(152.0)
    client.cycle()
    targets = [e["position"] for e in client.take_events() if e.get("what") == "target"]
    assert targets == [151.0, 152.0]


def test_bus_health_counts_lost_cycles_and_the_longest_loss(csp):
    client, transport, _ = csp
    client.enable()
    client.run(0.01)
    assert client.health() == {
        "bus_cycle_s": 0.001, "cycles": 10, "lost_cycles": 0, "longest_loss_cycles": 0,
        "losing_now": False,
    }  # fmt: skip
    transport.connected = False
    for _ in range(7):
        with pytest.raises(ConnectionError):
            client.cycle()
    assert client.health()["losing_now"]
    transport.connected = True
    client.cycle()
    health = client.health()
    assert (health["lost_cycles"], health["longest_loss_cycles"], health["losing_now"]) == (
        7,
        7,
        False,
    )
    bus = [e for e in client.take_events() if e["event"] == "bus"]
    assert [e["what"] for e in bus] == ["no answer from the drive", "the drive answers again"]
    assert bus[1]["after_cycles"] == 7


def test_the_status_carries_the_controlword_that_was_sent(pp):
    client, _, _ = pp
    client.enable()
    assert client.cycle().controlword == o.CMD_SHUTDOWN
    client.run(0.01)
    assert client.status.controlword == o.CMD_ENABLE_OPERATION


def test_the_event_buffer_is_bounded():
    client, _, _ = make("cyclic synchronous position")
    client.enable()
    client.run(0.01)
    for n in range(6000):
        client.move_to(150.0 + (n % 2))
    assert len(client.take_events()) == 4096
