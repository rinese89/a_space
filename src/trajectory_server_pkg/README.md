# trajectory_server_pkg

## Priority arbitration

Lower numerical value means higher priority.

When a candidate has a spatial conflict with an already active trajectory:

- active has strictly higher priority: candidate is rejected;
- equal priority: candidate waits for the active trajectory to finish;
- candidate has strictly higher priority: candidate also waits.

An already executing operation is never interrupted.

## Strictly higher-priority delayed candidate

Before changing the candidate's `operation_start_utc`, the server registers the
**original** trajectory in `trajectory_endtime_adjustment_node`.

The server waits for the service acknowledgement before the rescheduled copy
can enter `/active_trajectories`.

After the blocker finishes:

```text
new_start = blocker_real_completion + reschedule_margin_s
```

The server does **not** calculate `extra_time`.

The calendar delay:

```text
new_start - original_start
```

is also **not** `extra_time`.

`trajectory_endtime_adjustment_node` waits for the rescheduled active
occurrence to finish and computes:

```text
extra_time =
    real_duration_of_delayed_active_occurrence
    -
    original_duration
```

Thus delaying a 60-second trajectory by 25 seconds while it still lasts
60 seconds produces `extra_time = 0`.
