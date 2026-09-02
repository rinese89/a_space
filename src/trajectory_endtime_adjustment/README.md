# trajectory_endtime_adjustment

## Normal trajectory

When a normal occurrence disappears from `/active_trajectories`:

```text
extra_time = real_end - planned_operation_end_utc
```

The value is signed and may be positive, zero or negative.

## Delayed priority trajectory

Before `trajectory_server_node` changes the start of a strictly higher-priority
candidate, it sends the **original** trajectory to:

```text
/trajectory_endtime_adjustment/register_original_trajectory
```

The service only stores the original trajectory. It does not calculate
`extra_time`.

When the delayed copy appears in `/active_trajectories`, the node stores its
delayed `operation_start_utc`. It waits until that concrete active occurrence
disappears and uses the disappearance time as its real `END_active`.

The special correction is based only on the difference in durations:

```text
duration_original =
    END_original - START_original

duration_active =
    END_active_real - START_active_delayed

extra_time =
    duration_active - duration_original
```

The calendar delay of `START_active` has no direct effect on `extra_time`.

Output:

```text
START_adjusted = START_original
END_adjusted   = END_original + extra_time
extra_time     = duration_active - duration_original
```

Examples:

```text
Original:
10:05:00 -> 10:06:00 = 60 s

Active:
10:05:25 -> 10:06:25 = 60 s

extra_time = 0 s
Adjusted = 10:05:00 -> 10:06:00
```

and:

```text
Original:
10:05:00 -> 10:06:00 = 60 s

Active:
10:05:25 -> 10:06:35 = 70 s

extra_time = +10 s
Adjusted = 10:05:00 -> 10:06:10
```

A shorter real active duration yields a negative `extra_time`.
