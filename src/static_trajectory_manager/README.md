# static_trajectory_manager 0.6.0

Breaking message revision: one `StaticTrajectory` can now contain several
independent mission polylines.

## Message change

Previous field:

```text
static_trajectory_manager/TrajectorySegment mission
```

New field:

```text
static_trajectory_manager/TrajectorySegment[] mission
```

`takeoff` and `landing` remain single `TrajectorySegment` fields.

In generated C++:

```cpp
trajectory.takeoff
trajectory.mission[0]
trajectory.mission[1]
...
trajectory.landing
```

## Geometry semantics

Every `mission[i]` is an independent continuous polyline.

There is **no implicit geometric segment** between:

```text
mission[i].back()
and
mission[i+1].front()
```

This is intentional. It allows supervised cropped trajectories such as:

```text
A -> B

E -> F
```

to remain disconnected instead of becoming the false geometry:

```text
A -> B -> E -> F
```

### Partial repetitions

`repetitions` applies to the complete mission collection.

For geometric distance:

```text
distance =
    length(takeoff)
  + repetitions * sum(length(mission[i]))
  + length(landing)
```

No connector distances are added between independent mission elements.

## YAML

### New multi-mission syntax

Use a YAML sequence:

```yaml
mission:
  - x: [3.0, 4.0, 5.0]
    y: [6.0, 6.0, 6.0]
    z: [3.0, 3.0, 3.0]

  - x: [8.0, 9.0]
    y: [6.0, 6.0]
    z: [3.0, 3.0]
```

Do **not** repeat `x`, `y`, `z` keys inside one YAML map. Repeated YAML keys do
not represent several missions.

### Backward-compatible input shorthand

The node still accepts the old YAML form:

```yaml
mission:
  x: [3.0, 4.0, 5.0]
  y: [6.0, 6.0, 6.0]
  z: [3.0, 3.0, 3.0]
```

It is converted internally to an array containing one mission.

The ROS message itself is always the new array representation.

## Flight-zone validation

Each mission polyline is validated independently against its assigned
INCLUSION flight zone.

Only edges explicitly contained inside each mission are checked.

No virtual connector is checked between separate missions.

## `/adjusted_trajectories`

The geometry-update path has also been adapted.

The node now compares and replaces:

```text
takeoff
mission[]
landing
```

A supervision update can therefore replace one mission with several disconnected
mission fragments without reducing them to a single artificial polyline.

The existing DDS publisher-GID protection remains unchanged: geometry-only
heartbeats from the supervision publisher do not advance periodic repetitions,
while temporal adjustments from another publisher continue through the existing
periodic logic.

## RViz

TAKEOFF, each `mission[i]`, and LANDING are emitted as separate `LINE_STRIP`
markers.

This prevents RViz from drawing artificial connections between mission
components.

## Breaking downstream API

Packages compiled against the previous message will need adaptation.

Old code:

```cpp
trajectory.mission.x
trajectory.mission.y
trajectory.mission.z
```

New code:

```cpp
for (const auto & mission : trajectory.mission) {
  mission.x;
  mission.y;
  mission.z;
}
```

Likely downstream packages include the static conflict manager, virtual net,
collision/deconfliction/supervision path, trajectory server and control path.
They should be migrated one at a time.

## Build

Because the ROS interface changed, clean every package that directly or
transitively depends on `static_trajectory_manager` before rebuilding.

At minimum:

```bash
cd ~/a_space_ws

rm -rf build/static_trajectory_manager
rm -rf install/static_trajectory_manager

colcon build --symlink-install \
  --packages-select static_trajectory_manager

source install/setup.bash
```

After downstream packages are adapted, rebuild the complete dependency chain.

## Inspect the new interface

```bash
ros2 interface show static_trajectory_manager/msg/StaticTrajectory
```

The mission field must show:

```text
static_trajectory_manager/TrajectorySegment[] mission
```
