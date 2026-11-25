# Changelog

## 0.1.0 (not released)

- An emulated CiA 402 servo drive: object dictionary, power state machine, profile position,
  cyclic synchronous position, homing to the present position, a lagging axis, following error
  and limit supervision, quick stop, faults, and a process-data watchdog.
- `Cia402Client`, the host side, over a `Transport`. `EmulatedTransport` reaches the emulated
  drive. No transport for a real bus is included.
- For a record of a run: the drive's identity, every setting written with its read-back,
  changes at bus rate as events, bus health, and the controlword sent.
- Seeded chaos and stress tests. `FEATURES.md` lists each supported scenario with its test.
