# Geometry-only crop of one trajectory on the A-space virtual net.
#
# This is deliberately NOT a StaticTrajectory/TrajectorySegment: deleting a
# middle grid section can create disconnected pieces, which a single x/y/z
# polyline cannot represent without reconnecting across the removed gap.

string trajectory_id
string frame_id
uint32 source_repetitions

# True when /net_loaded_trajectories contained the complete discretized route.
# False means the detected collision is preserved but the crop could not yet
# be constructed.
bool complete

# Classification count: unique collision node IDs only.
uint32 unique_collision_node_count

# All non-collision lattice nodes, represented once geometrically.
deconfliction_manager/CroppedNetNode[] retained_nodes

# All retained lattice edges. Repeated partial mission traversals of the same
# (phase, edge_id) are represented only once.
a_space_virtual_net/NetTrajectorySegment[] retained_segments

# Collision nodes removed from the route.
uint64[] removed_node_ids

# Removed edges. This includes:
#   - edges explicitly reported as collision_segments, and
#   - every edge incident to a removed collision node.
a_space_virtual_net/NetTrajectorySegment[] removed_segments
uint64[] removed_edge_ids
