# Exact geometry crop produced by deconfliction_manager_node.

string trajectory_id
string frame_id
uint32 source_repetitions

# True when the matching /net_loaded_trajectories entry was available and the
# virtual-net crop could be computed.
bool complete

# True when the crop can also be represented exactly by the current
# StaticTrajectory schema. mission[] can contain any number of disconnected
# retained MISSION fragments. TAKEOFF and LANDING are still single polylines,
# so this is false only if either of those phases is split into >1 retained run.
bool trajectory_materialized

# Ready-to-use cropped StaticTrajectory.
# It preserves all metadata from the original trajectory and replaces geometry:
#   - takeoff: retained takeoff run or empty
#   - mission[]: one entry per retained continuous mission fragment
#   - landing: retained landing run or empty
static_trajectory_manager/StaticTrajectory trajectory

# Classification count: unique collision node IDs only.
uint32 unique_collision_node_count

# All non-collision lattice nodes, represented once physically.
deconfliction_manager/CroppedNetNode[] retained_nodes

# Geometry-only lattice edges. Partial repetitions are collapsed, but
# different mission[] components are never merged together.
deconfliction_manager/CroppedNetSegment[] retained_segments

uint64[] removed_node_ids

# Removed edges include explicit collision edges plus every edge incident to a
# collision node. Same geometric edge may appear once per distinct mission[].
deconfliction_manager/CroppedNetSegment[] removed_segments
uint64[] removed_edge_ids
