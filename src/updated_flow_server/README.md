# updated_flow_server

ROS 2 Humble package containing `updated_flow_server_node`.

## Inputs

- `/manual_adjustment_trajectories` (`StaticTrajectoryArray`)
- `/solved_collision_trajectories` (`StaticTrajectoryArray`)
- `/collision_static_trajectories` (`CollisionStaticTrajectoryArray`)
- `/available_static_trajectories` (`StaticTrajectoryArray`)

All subscriptions use `RELIABLE + TRANSIENT_LOCAL + KeepLast(1)`.

## `/removed_trajectories`

Type: `static_trajectory_manager/msg/StaticTrajectoryArray`.

It republishes the complete authoritative manual-adjustment snapshot. These are the trajectories that must be removed from the automatic flow.

A manual trajectory is verified as removed when it is absent from both:

- `/available_static_trajectories`
- `/collision_static_trajectories`

## `/updated_flow_status`

Type: `updated_flow_server/msg/UpdatedFlowStatus`.

For every trajectory currently present in the manual or solved snapshots it reports:

- expected state (`EXPECT_REMOVED` or `EXPECT_AVAILABLE`)
- whether it is currently present in available
- whether it is currently present in collision
- whether the expected state is verified
- whether the input is contradictory
- a textual detail

A solved trajectory is verified when:

```text
present in /available_static_trajectories
AND
absent from /collision_static_trajectories
```

A removed/manual trajectory is verified when:

```text
absent from /available_static_trajectories
AND
absent from /collision_static_trajectories
```

If the same ID appears simultaneously in manual and solved, removal takes precedence and the status is marked contradictory.

## Publication

`/removed_trajectories` and `/updated_flow_status` use `RELIABLE + TRANSIENT_LOCAL + KeepLast(1)` and are republished every second by default. They are also published immediately when any input snapshot changes.

## Topic name note

This package follows the requested solved topic name:

```text
/solved_collision_trajectories
```

If the current `deconfliction_manager_node` still publishes `/solved_deconfliction_trajectories`, set:

```yaml
solved_collision_trajectories_topic: /solved_deconfliction_trajectories
```

or rename the upstream output.

## Build

```bash
cd ~/a_space_ws
rm -rf build/updated_flow_server install/updated_flow_server
colcon build --symlink-install --packages-up-to updated_flow_server
source install/setup.bash
```

## Launch

```bash
ros2 launch updated_flow_server updated_flow_server.launch.py
```

## Inspect

```bash
ros2 topic echo /removed_trajectories
ros2 topic echo /updated_flow_status
```
