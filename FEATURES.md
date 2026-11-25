# Features

Each feature is a scenario the package supports, with the test that shows it. "Client" means
`Cia402Client`, the host side that also drives a real drive. "Drive" means `EmulatedDrive`.

Positions are in increments at the drive and in the host's unit (for example mm) at the client.

## Starting, stopping and power states

| Feature | What happens | Shown by |
|---|---|---|
| Enable | The client walks the drive through shutdown, switch on and enable operation, one state per bus cycle, acting on the state the drive reports | `test_enabling_walks_the_state_machine_one_state_per_cycle` |
| Every power transition | Each controlword command causes its CiA 402 transition (2 to 10), and the statusword states the result | `test_each_command_causes_its_transition` |
| No skipped states | Enable operation sent to a drive that is switch on disabled does nothing | `test_enable_operation_is_not_taken_from_switch_on_disabled` |
| Disable | The power stage goes off, the axis holds, and the target is dropped. A later enable does not resume the old move | `test_disabling_holds_the_axis_and_drops_the_target` |
| Settings locked while enabled | The client refuses to change drive settings while the drive is enabled | `test_settings_cannot_be_changed_while_enabled` |

## Profile position (mode 1): the drive runs the trajectory

| Feature | What happens | Shown by |
|---|---|---|
| A move | A trapezoid: acceleration, the profile velocity, deceleration, and the target reached bit when within the position window | `test_a_move_follows_a_trapezoid_and_ends_on_target` |
| Set-point handshake | The client raises new set-point, the drive acknowledges, the client lowers it | `test_the_set_point_handshake_raises_bit_4_until_acknowledged_then_lowers_it` |
| A new target mid-move | The new target replaces the old one at once | `test_a_second_target_while_moving_replaces_the_first` |
| Following error | The axis lags the profile by the speed times the position loop's time constant | `test_the_axis_lags_the_profile_by_the_speed_times_the_loops_time_constant` |
| A target beyond the travel | Held to the software limit, with internal limit active set | `test_a_target_beyond_the_travel_is_held_to_the_limit_and_the_drive_says_so` |

Also in the drive, without a client call for them: one buffered set-point, relative moves, and
halt.

## Cyclic synchronous position (mode 8): the host streams the trajectory

| Feature | What happens | Shown by |
|---|---|---|
| Following a stream | A target every bus cycle is tracked within the position loop's lag | `test_a_streamed_sine_is_tracked_within_the_loops_lag` |
| A jump in the target | Rate-limited to the drive's maximum velocity, with internal limit active set. The profile velocity and acceleration do not apply in this mode | `test_a_step_larger_than_the_drive_can_travel_in_a_cycle_is_rate_limited` |
| A slow host above a fast bus | A target that changes at 10 Hz over a 1 ms bus gives monotonic motion | `test_a_host_updating_at_10_hz_gives_a_staircase_the_drive_smooths` |

## Homing (mode 6)

| Feature | What happens | Shown by |
|---|---|---|
| Home here | The present position becomes the home position (methods 35 and 37) | `test_homing_makes_the_present_position_the_home_position` |

Other homing methods end in homing error.

## Stops and faults

| Feature | What happens | Shown by |
|---|---|---|
| Quick stop | The drive decelerates on the quick stop ramp and ends switch on disabled | `test_quick_stop_decelerates_on_its_own_ramp_and_ends_switch_on_disabled` |
| Quick stop and stay | With option code 5 or 6 the drive stays in quick stop active and can be enabled again, with no old target | `test_with_option_code_6_the_drive_stays_in_quick_stop_and_can_be_enabled_again` |
| A stalled axis | The following error leaves its window for longer than its time out, the drive faults with error 0x8611 and holds | `test_a_stalled_axis_faults_on_following_error_and_holds` |
| A lost bus | No process data for the watchdog time faults the drive with error 0x7500, and it stops | `test_a_lost_bus_faults_the_drive_after_its_watchdog_time_and_it_stops` |
| A stalled host | A host that stops cycling is a lost bus to the drive | `test_a_host_that_stops_cycling_is_a_lost_bus_to_the_drive` |
| Fault reset | A fault leaves only on a rising edge of fault reset, and the drive comes back disabled | `test_a_fault_leaves_only_on_a_rising_edge_of_fault_reset`, `test_a_fault_needs_a_reset_and_then_a_fresh_enable` |
| Reset during the fault reaction | A drive ignores a reset while it is still ramping down. The client holds the request and sends it once the drive reports fault | `test_a_reset_asked_for_during_the_fault_reaction_is_held_until_the_drive_can_take_it` |

## The object dictionary

| Feature | What happens | Shown by |
|---|---|---|
| Read-only objects | A write is refused with abort 0x06010002 | `test_a_read_only_object_cannot_be_written` |
| Unknown objects | Refused with 0x06020000, or 0x06090011 for a subindex | `test_an_unknown_object_or_subindex_is_named` |
| Typed values | A value outside the object's type is refused with 0x06090030 | `test_a_value_outside_the_objects_type_is_refused` |
| Supported modes | Object 0x6502 states profile position, homing and cyclic synchronous position | `test_the_drive_states_which_modes_it_supports` |

## What holds under random and extreme use

These are in `tests/test_chaos.py`. Every case is seeded.

| Guarantee | Shown by |
|---|---|
| Random controlwords, modes and targets: the statusword always states the power state, the axis moves only with the power stage on, stays inside its limits and under the maximum velocity, and every object stays inside its type | `test_random_controlwords_modes_and_targets_never_break_the_drive` (40 seeds) |
| No controlword other than a fault reset edge leaves fault | `test_a_fault_is_left_only_by_a_rising_edge_of_fault_reset` (20 seeds) |
| A velocity, acceleration or deceleration of 0, or a window of 0 or its maximum, never hangs the drive. A deceleration of 0 stops at once | `test_an_extreme_setting_never_hangs_the_drive_or_gives_a_bad_number` |
| Any position loop time constant from 0 to 5 s and any bus cycle from 10 µs to 100 ms gives finite, bounded motion | `test_any_loop_time_constant_and_bus_cycle_gives_finite_bounded_motion` |
| Software limits written with the minimum above the maximum limit nothing | `test_inverted_limits_written_raw_do_not_trap_the_axis` |
| Random client calls with stalls and disconnects raise only the client's refusals and a lost connection, and with the faults gone the client always gets a move finished | `test_random_use_of_the_client_raises_only_its_own_refusals_and_always_recovers` (30 seeds, both modes) |
| Five minutes of bus time does not drift, and a cycle takes well under 0.5 ms of computer time | `test_a_long_run_does_not_drift_and_stays_fast` |
| A run repeats to the count | `test_a_run_is_repeatable_to_the_count` |

A one-off sweep of 1,400 further seeds across the three random cases found no failure.

## Defects the chaos tests found, now fixed

1. After a stall cleared, the axis caught up with its demand faster than the drive's velocity
   limit. The axis is now held to that limit.
2. A quick stop or fault reaction with a deceleration of 0 never ended. It now stops at once.
3. A fault reset sent while the drive was still ramping down was ignored, as a real drive
   ignores it, and the client did not retry. The client now holds the request until the drive
   reports fault.
4. A fault reset asked for before the client had read the fault was dropped. It is now kept
   until a status read since the request shows no fault.

## Not supported

- Velocity and torque modes, interpolated position mode.
- Homing on switches or an index pulse.
- Torque, current, the motor, the brake, temperature.
- PDO mapping objects: the process data is a fixed set (see `transport.py`).
- Any manufacturer-specific object or behaviour.
- A transport for a real bus. The client is written for one; none is in this package.
- A host-side trajectory generator for cyclic synchronous position. In that mode a jump in
  the target is taken at the drive's maximum velocity.
