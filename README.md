# axiom-ext-cia402

An emulated CiA 402 servo drive, and the host-side client that drives it or a real drive.
No dependencies.

CiA 402 is the CANopen device profile for drives and motion control (IEC 61800-7-201). Servo
drives on EtherCAT carry it as CoE. A host commands such a drive by writing objects (the
controlword, the mode of operation, a target) and reading others back (the statusword, the
actual position).

The point of this package is that the host code is written once. `Cia402Client` talks to a
`Transport`. With `EmulatedTransport` it reaches an `EmulatedDrive` in the same process, on
simulated time. With a transport over a real EtherCAT master it reaches a real drive. The
client, its settings and the tests of its behaviour stay the same.

```python
from axiom_ext_cia402 import Cia402Client, EmulatedDrive, EmulatedTransport, Scaling

client = Cia402Client(EmulatedTransport(EmulatedDrive(), cycle_s=0.001), Scaling(1000.0, "mm"))
client.configure(mode="profile position", limits=(0.0, 305.0), profile_velocity=10.0)
client.enable()
client.run(0.01)
client.move_to(5.0)
status = client.run(2.0)   # status.position == 5.0, status.target_reached is True
```

## Name

The repository is `axiom-ext-cia402`, by Axiom's rule for a standalone extension specific to
one capability (`axiom-ext-<code-or-capability>`, kept in the consuming organization). The
Python package has the same name, `axiom_ext_cia402`, as the organization's other extensions
do. It has no dependency on Axiom, so that a machine
whose software must have no dependencies can install it. An AEOS manifest
(`axiom-extension.toml`) is not written yet.

## What the emulated drive does

| Part of the profile | Emulated |
|---|---|
| Object dictionary | Typed objects, read-only objects, SDO abort codes |
| Power state machine | All eight states and the transitions a controlword causes; fault reset on a rising edge |
| Profile position (mode 1) | Trapezoidal profile, set-point handshake, one buffered set-point, change immediately, relative moves, halt, target reached |
| Cyclic synchronous position (mode 8) | A target every cycle, rate-limited by the maximum profile velocity |
| Homing (mode 6) | Methods 35 and 37 only: the present position becomes home |
| The axis | The actual position lags the demand by a first-order position loop, so there is a following error |
| Supervision | Following error window and time out, software position limits, internal limit active |
| Stops and faults | Quick stop option codes 0, 1, 2, 5, 6; fault reaction; error codes |
| The bus | A process-data watchdog: no cycle for the watchdog time faults the drive |

Not emulated: torque and current, the motor and brake, velocity and torque modes, the other
homing methods, touch probes, PDO mapping objects, and any manufacturer-specific object.

## What is assumed

Everything about the axis is a placeholder until it is measured on a real drive:
the position loop's time constant (20 ms), the watchdog time (100 ms), and every default in
the object dictionary. Where a manufacturer's drive differs from the profile, or adds to it,
this emulator does not know. Compare it against the drive's own object list before relying
on it for that drive.

## Status

Version 0.1.0, not released to any package index. The emulator is built from the CiA 402
profile in general. It has not been compared against any manufacturer's drive or object list,
and it has not been run against hardware. Treat every default as a placeholder.

## Install

From a checkout:

```
pip install .
python -m axiom_ext_cia402     # exercises an emulated drive and prints PASS or what failed
```

It needs Python 3.11 or newer and nothing else.

## Tests

```
pip install -e ".[dev]"
pytest
```

A run is deterministic: the same calls give the same positions to the count.

## Licence

Apache-2.0. See `LICENSE`.
