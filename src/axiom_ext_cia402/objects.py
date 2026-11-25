# Copyright (c) 2026 The University of Texas at Austin
# Copyright (c) 2026 B-Tree Labs
# SPDX-License-Identifier: Apache-2.0
"""The CiA 402 objects, bits and states this package uses, by their names in the profile.

CiA 402 is the CANopen device profile for drives and motion control (IEC 61800-7-201). Over
EtherCAT it is carried as CoE. Positions are in increments, velocities in increments per
second and accelerations in increments per second squared, as the drive holds them.
"""

from __future__ import annotations

from dataclasses import dataclass

# ---- object indices -------------------------------------------------------------------------

DEVICE_TYPE = 0x1000
IDENTITY = 0x1018  # sub 1 vendor id, 2 product code, 3 revision number, 4 serial number
ERROR_CODE = 0x603F
CONTROLWORD = 0x6040
STATUSWORD = 0x6041
QUICK_STOP_OPTION_CODE = 0x605A
FAULT_REACTION_OPTION_CODE = 0x605E
MODES_OF_OPERATION = 0x6060
MODES_OF_OPERATION_DISPLAY = 0x6061
POSITION_DEMAND_VALUE = 0x6062
POSITION_ACTUAL_VALUE = 0x6064
FOLLOWING_ERROR_WINDOW = 0x6065
FOLLOWING_ERROR_TIME_OUT = 0x6066
POSITION_WINDOW = 0x6067
POSITION_WINDOW_TIME = 0x6068
VELOCITY_ACTUAL_VALUE = 0x606C
TARGET_POSITION = 0x607A
HOME_OFFSET = 0x607C
SOFTWARE_POSITION_LIMIT = 0x607D  # sub 1 minimum, sub 2 maximum
MAX_PROFILE_VELOCITY = 0x607F
PROFILE_VELOCITY = 0x6081
PROFILE_ACCELERATION = 0x6083
PROFILE_DECELERATION = 0x6084
QUICK_STOP_DECELERATION = 0x6085
HOMING_METHOD = 0x6098
INTERPOLATION_TIME_PERIOD = 0x60C2  # sub 1 value, sub 2 index (a power of ten, in seconds)
FOLLOWING_ERROR_ACTUAL_VALUE = 0x60F4
SUPPORTED_DRIVE_MODES = 0x6502

# ---- modes of operation (0x6060) ------------------------------------------------------------

NO_MODE = 0
PROFILE_POSITION = 1
HOMING = 6
CYCLIC_SYNCHRONOUS_POSITION = 8
MODE_NAMES = {
    NO_MODE: "no mode",
    PROFILE_POSITION: "profile position",
    HOMING: "homing",
    CYCLIC_SYNCHRONOUS_POSITION: "cyclic synchronous position",
}
MODE_CODES = {name: code for code, name in MODE_NAMES.items()}
# 0x6502: bit 0 pp, bit 5 hm, bit 7 csp
SUPPORTED_MODES_BITS = (1 << 0) | (1 << 5) | (1 << 7)

# ---- controlword (0x6040) -------------------------------------------------------------------

CW_SWITCH_ON = 1 << 0
CW_ENABLE_VOLTAGE = 1 << 1
CW_QUICK_STOP = 1 << 2  # 0 asks for a quick stop
CW_ENABLE_OPERATION = 1 << 3
CW_NEW_SET_POINT = 1 << 4  # profile position; homing: start
CW_CHANGE_SET_IMMEDIATELY = 1 << 5  # profile position
CW_RELATIVE = 1 << 6  # profile position: 0 absolute, 1 relative
CW_FAULT_RESET = 1 << 7  # acts on its rising edge
CW_HALT = 1 << 8

CMD_SHUTDOWN = CW_ENABLE_VOLTAGE | CW_QUICK_STOP  # 0x06
CMD_SWITCH_ON = CMD_SHUTDOWN | CW_SWITCH_ON  # 0x07
CMD_ENABLE_OPERATION = CMD_SWITCH_ON | CW_ENABLE_OPERATION  # 0x0F
CMD_DISABLE_VOLTAGE = 0x00
CMD_QUICK_STOP = CW_ENABLE_VOLTAGE  # 0x02

# ---- statusword (0x6041) --------------------------------------------------------------------

SW_READY_TO_SWITCH_ON = 1 << 0
SW_SWITCHED_ON = 1 << 1
SW_OPERATION_ENABLED = 1 << 2
SW_FAULT = 1 << 3
SW_VOLTAGE_ENABLED = 1 << 4
SW_QUICK_STOP = 1 << 5  # 0 while a quick stop is active
SW_SWITCH_ON_DISABLED = 1 << 6
SW_WARNING = 1 << 7
SW_REMOTE = 1 << 9
SW_TARGET_REACHED = 1 << 10
SW_INTERNAL_LIMIT_ACTIVE = 1 << 11
SW_BIT_12 = 1 << 12  # pp: set-point acknowledge; csp: drive follows the command; hm: attained
SW_BIT_13 = 1 << 13  # pp, csp: following error; hm: homing error

# ---- the power state machine ----------------------------------------------------------------

NOT_READY_TO_SWITCH_ON = "not ready to switch on"
SWITCH_ON_DISABLED = "switch on disabled"
READY_TO_SWITCH_ON = "ready to switch on"
SWITCHED_ON = "switched on"
OPERATION_ENABLED = "operation enabled"
QUICK_STOP_ACTIVE = "quick stop active"
FAULT_REACTION_ACTIVE = "fault reaction active"
FAULT = "fault"

# The statusword bits that state each power state: (mask, value).
STATE_CODES = {
    NOT_READY_TO_SWITCH_ON: (0x4F, 0x00),
    SWITCH_ON_DISABLED: (0x4F, 0x40),
    READY_TO_SWITCH_ON: (0x6F, 0x21),
    SWITCHED_ON: (0x6F, 0x23),
    OPERATION_ENABLED: (0x6F, 0x27),
    QUICK_STOP_ACTIVE: (0x6F, 0x07),
    FAULT_REACTION_ACTIVE: (0x4F, 0x0F),
    FAULT: (0x4F, 0x08),
}


def state_of(statusword: int) -> str:
    """The power state a statusword states."""
    for name, (mask, value) in STATE_CODES.items():
        if statusword & mask == value:
            return name
    raise ValueError(f"statusword {statusword:#06x} states no CiA 402 power state")


# ---- error codes (0x603F) this emulator raises -----------------------------------------------

ERR_NONE = 0x0000
ERR_COMMUNICATION = 0x7500  # no process data within the watchdog time
ERR_FOLLOWING = 0x8611  # following error
ERROR_NAMES = {
    ERR_NONE: "no error",
    ERR_COMMUNICATION: "communication lost (process-data watchdog)",
    ERR_FOLLOWING: "following error",
}

# ---- data types -----------------------------------------------------------------------------


@dataclass(frozen=True)
class Type:
    name: str
    low: int
    high: int


I8 = Type("INTEGER8", -(2**7), 2**7 - 1)
I16 = Type("INTEGER16", -(2**15), 2**15 - 1)
I32 = Type("INTEGER32", -(2**31), 2**31 - 1)
U8 = Type("UNSIGNED8", 0, 2**8 - 1)
U16 = Type("UNSIGNED16", 0, 2**16 - 1)
U32 = Type("UNSIGNED32", 0, 2**32 - 1)
