# static_trajectory_conflict_manager

ROS 2 Humble package implementing the simplified **first stage** of static
trajectory conflict management for the A-space.

## New responsibility

This node now answers only one question:

```text
Does this requested trajectory collide with the already accepted
collision-free trajectory set?
```

It does **not** classify the collision type and does **not** generate collision
geometry/evidence.

The downstream collision-classification node is responsible for that work.

## Authoritative input

```text
/requested_static_trajectories
```

This is treated as a retained authoritative snapshot.

A change means:

- a new `trajectory_id`;
- a removed `trajectory_id`;
- or any field/geometry/time change in an existing trajectory.

Unchanged snapshots do not trigger another collision computation.

## Evaluation order

The current requested snapshot is rebuilt deterministically in:

```text
priority ascending
trajectory_id ascending
```

Lower numerical priority therefore enters the accepted comparison set first.

Priority is used only as an **evaluation order**. The previous complex
priority/blocker/equal-priority arbitration has been removed.

## Classification algorithm

For every requested trajectory:

```text
candidate
   |
   +--> compare against accepted trajectory 1
   |        |
   |        +--> time windows cannot overlap -> next accepted
   |        |
   |        +--> inspect finite segment pairs
   |                   |
   |                   +--> distance > minimum_separation_m -> continue
   |                   |
   |                   +--> FIRST collision found
   |                              |
   |                              +--> STOP analysis immediately
   |                              +--> classify candidate COLLISION
   |
   +--> if no accepted trajectory collides
              |
              +--> classify AVAILABLE
              +--> insert into accepted comparison set
```

Only AVAILABLE trajectories are used to test later candidates.

A COLLISION trajectory is never used as a blocker/reference trajectory.

## Why the current snapshot is rebuilt when input changes

Only the accepted set is operational state.

However, if an accepted trajectory disappears, a trajectory previously
classified as collision may become free. Rebuilding from the current
authoritative `/requested_static_trajectories` snapshot guarantees that the
classification is immediately consistent after additions, removals or
modifications.

The node does **not** keep detailed collision evidence or blocker caches.

## Spatial collision

Before geometry is inspected, complete operation time windows must be capable
of overlapping. Periodic operation definitions continue to be handled
analytically.

Spatially, the node expands:

```text
TAKEOFF -> MISSION x repetitions -> LANDING
```

and walks the candidate geometry in order.

For each candidate segment, it walks the accepted trajectory segments.

The instant:

```text
segment_distance <= minimum_separation_m
```

becomes true, the pair test returns `true`.

No subsequent segment is examined for that candidate/accepted pair.

The node does not calculate:

- collision phase;
- segment index;
- closest points to publish;
- number of collisions;
- superposition;
- intersection;
- VTOL collision class.

## Outputs

### Collision-free trajectories

```text
/available_static_trajectories
```

Type:

```text
static_trajectory_manager/msg/StaticTrajectoryArray
```

This is the stored comparison set.

### Collision trajectories

```text
/collision_static_trajectories
```

Type remains:

```text
static_trajectory_conflict_manager/msg/CollisionStaticTrajectoryArray
```

for compatibility with the current A-space interfaces.

Each element contains:

```text
trajectory
```

but:

```text
collisions[]
```

is deliberately empty.

**Presence in `/collision_static_trajectories` is itself the collision
classification.**

The downstream collision classifier must therefore no longer expect this first
node to provide `SegmentCollision` evidence.

## Persistence

Both outputs use:

```text
RELIABLE
TRANSIENT_LOCAL
KeepLast(1)
```

and both are republished periodically.

Default:

```yaml
publish_period_ms: 1000
```

So:

```text
/available_static_trajectories
/collision_static_trajectories
```

are complete authoritative snapshots at 1 Hz.

Empty snapshots are also published, which lets consumers clear stale state.

The collision trajectories are retained only in an output-snapshot cache so
they can be republished. They are **never** part of the comparison set.

## Markers

The node also retains:

```text
/available_static_trajectories_markers
/collision_static_trajectories_markers
```

Available trajectories are blue.

Collision trajectories are red.

Only the complete route is drawn; there are no collision-point markers because
this first-stage node intentionally no longer calculates collision evidence.

## Parameters

```yaml
minimum_separation_m: 1.0
use_3d: true
publish_period_ms: 1000
```

## Build

```bash
cd ~/a_space_ws

rm -rf build/static_trajectory_conflict_manager
rm -rf install/static_trajectory_conflict_manager

colcon build --symlink-install \
  --packages-select static_trajectory_conflict_manager

source install/setup.bash
```

## Important downstream change

The previous `static_trajectory_collision_classifier` consumed detailed
`TrajectoryCollision/SegmentCollision` evidence generated here.

That evidence is intentionally absent now.

The next revision of the classifier must:

1. read `/collision_static_trajectories`;
2. obtain the accepted/reference trajectories from
   `/available_static_trajectories` or the authoritative requested set;
3. compute the actual collision segments itself;
4. perform VTOL/intersection/superposition classification.

That separation is consistent with the new architecture:

```text
conflict manager
    -> boolean collision / no collision

collision classifier
    -> detailed collision geometry and type
```
