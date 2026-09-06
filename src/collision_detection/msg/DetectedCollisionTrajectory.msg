# Collision trajectory enriched with exact virtual-net resources shared with
# other loaded trajectories.
#
# `trajectory` is the complete multi-mission StaticTrajectory. This node does
# not modify takeoff, mission[] or landing.

static_trajectory_manager/StaticTrajectory trajectory

# False when the trajectory is present in /collision_static_trajectories but a
# matching trajectory_id has not yet appeared in /net_loaded_trajectories.
bool loaded_on_net

# True when at least one shared node or edge was found on the discretized net.
bool has_shared_net_space

# Union of all other trajectory IDs sharing at least one marked node or edge.
string[] conflicting_trajectory_ids

collision_detection/GridCollisionNode[] collision_nodes
collision_detection/GridCollisionSegment[] collision_segments
