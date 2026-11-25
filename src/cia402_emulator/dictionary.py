# Copyright (c) 2026 The University of Texas at Austin
# SPDX-License-Identifier: Apache-2.0
"""An object dictionary: typed entries read and written by index and subindex.

A write to a read-only object, to an object that does not exist, or of a value outside the
object's type is refused with the SDO abort code a drive would return.
"""

from __future__ import annotations

from dataclasses import dataclass

from . import objects as o

ABORT_NO_OBJECT = 0x06020000
ABORT_NO_SUBINDEX = 0x06090011
ABORT_READ_ONLY = 0x06010002
ABORT_OUT_OF_RANGE = 0x06090030
ABORT_NAMES = {
    ABORT_NO_OBJECT: "object does not exist in the object dictionary",
    ABORT_NO_SUBINDEX: "subindex does not exist",
    ABORT_READ_ONLY: "attempt to write a read-only object",
    ABORT_OUT_OF_RANGE: "value range of parameter exceeded",
}


class SdoAbort(Exception):
    def __init__(self, code: int, index: int, subindex: int) -> None:
        self.code, self.index, self.subindex = code, index, subindex
        super().__init__(f"{index:#06x}:{subindex} {ABORT_NAMES[code]} (abort {code:#010x})")


@dataclass
class Entry:
    name: str
    type: o.Type
    writable: bool
    value: int


class ObjectDictionary:
    def __init__(self) -> None:
        self._entries: dict[tuple[int, int], Entry] = {}

    def add(self, index: int, name: str, type: o.Type, value: int = 0, *, writable: bool = True,
            subindex: int = 0) -> None:  # fmt: skip
        self._entries[(index, subindex)] = Entry(name, type, writable, value)

    def entry(self, index: int, subindex: int = 0) -> Entry:
        found = self._entries.get((index, subindex))
        if found is None:
            known = any(i == index for i, _ in self._entries)
            raise SdoAbort(ABORT_NO_SUBINDEX if known else ABORT_NO_OBJECT, index, subindex)
        return found

    def read(self, index: int, subindex: int = 0) -> int:
        return self.entry(index, subindex).value

    def write(self, index: int, subindex: int, value: int) -> None:
        """A write from the host, checked."""
        entry = self.entry(index, subindex)
        if not entry.writable:
            raise SdoAbort(ABORT_READ_ONLY, index, subindex)
        if isinstance(value, bool) or not isinstance(value, int):
            raise SdoAbort(ABORT_OUT_OF_RANGE, index, subindex)
        if not entry.type.low <= value <= entry.type.high:
            raise SdoAbort(ABORT_OUT_OF_RANGE, index, subindex)
        entry.value = value

    def set(self, index: int, value: int, subindex: int = 0) -> None:
        """The drive's own update of an object, held to the object's type."""
        entry = self.entry(index, subindex)
        entry.value = min(max(int(value), entry.type.low), entry.type.high)

    def items(self):
        return sorted(self._entries.items())
