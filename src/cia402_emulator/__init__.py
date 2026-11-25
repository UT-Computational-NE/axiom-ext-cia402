# Copyright (c) 2026 The University of Texas at Austin
# SPDX-License-Identifier: Apache-2.0
"""A CiA 402 servo drive emulator, and the host-side client that drives it or a real drive.

from cia402_emulator import Cia402Client, EmulatedDrive, EmulatedTransport, Scaling

client = Cia402Client(EmulatedTransport(EmulatedDrive(), cycle_s=0.001), Scaling(1000.0))
client.configure(mode="profile position", limits=(0.0, 305.0), profile_velocity=10.0)
client.enable()
client.run(0.01)
client.move_to(5.0)
status = client.run(2.0)      # status.position is 5.0 mm, status.target_reached is True
"""

from .client import Cia402Client, ClientRefused, DriveStatus, Scaling
from .dictionary import ObjectDictionary, SdoAbort
from .drive import AxisModel, EmulatedDrive
from .transport import EmulatedTransport, Transport

__version__ = "0.1.0"
__all__ = [
    "AxisModel",
    "Cia402Client",
    "ClientRefused",
    "DriveStatus",
    "EmulatedDrive",
    "EmulatedTransport",
    "ObjectDictionary",
    "Scaling",
    "SdoAbort",
    "Transport",
    "__version__",
]
