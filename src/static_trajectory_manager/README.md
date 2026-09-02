# static_trajectory_manager

Long-term store and validator for static UAS trajectories.

## Repetition model

Two different repetition concepts are deliberately kept separate:

- `repetitions`: **partial repetition**. It repeats only the `mission` segment
  inside one complete takeoff -> mission -> landing operation.
- `operation_frequency`: **total repetition period**, in seconds, between
  complete-operation starts.
- `total_repetitions`: YAML-only total-operation count:
  - `0`: unlimited complete repetitions (only valid/useful when
    `operation_frequency > 0`);
  - `N > 0`: exactly N complete operations.

`total_repetitions` is intentionally an internal long-term-store property and
is not added to `StaticTrajectory.msg`.

## Renewal of a periodic trajectory

The manager publishes only one concrete complete occurrence at a time.

When `/adjusted_trajectories` reports that occurrence as completed:

```text
actual_start = adjusted.operation_start_utc
actual_end   = adjusted.operation_end_utc
period       = operation_frequency

next_start = actual_start + period
next_end   = actual_end   + period
```

The adjusted end may represent a positive or negative `extra_time`. Shifting
both limits by the same period carries the measured duration correction into
the next occurrence.

This also handles an occurrence whose start was delayed by
`trajectory_server_node`: the adjusted occurrence start is authoritative.

## Internal counters

For each periodic trajectory the node maintains:

- `completed_total_repetitions`;
- remaining complete repetitions.

If `total_repetitions == 0`, remaining is reported internally as `unlimited`.

When a finite periodic trajectory reaches its last complete repetition, it is
no longer renewed and is moved to `/latest_trajectories` with the actual
start/end of the last occurrence.

The counters are runtime state and reset if the node is restarted.

## Topics

Subscriptions:

- `/flight_zones`
- `/adjusted_trajectories`

Publishers:

- `/requested_static_trajectories`
- `/unvalidated_trajectories`
- `/latest_trajectories`
- `/requested_static_trajectories_markers`

Only requested trajectories receive markers.

## Duplicate/stale adjustment protection

Each processed periodic occurrence is keyed internally by its concrete
`operation_start_utc`. A repeated retained adjustment is ignored. An
adjustment older than the currently stored occurrence is also ignored.

## Build

```bash
colcon build --symlink-install --packages-select static_trajectory_manager
source install/setup.bash
```
