# First-stage classification result.
# Presence of this message in /collision_static_trajectories means that the
# trajectory collided with at least one already accepted trajectory.
#
# `collisions` is intentionally left empty by the simplified first-stage
# conflict manager. Detailed collision geometry belongs to the downstream
# collision-classification stage.

static_trajectory_manager/StaticTrajectory trajectory
static_trajectory_conflict_manager/TrajectoryCollision[] collisions
