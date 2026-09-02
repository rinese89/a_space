# One target-trajectory lattice segment whose edge_id is also used by at least
# one other loaded trajectory.

uint64 edge_id
uint8 axis

uint64 start_node_id
uint64 end_node_id
geometry_msgs/Point start
geometry_msgs/Point end
float64 length_m

uint8 phase
uint32 original_phase_segment_index
uint32 net_segment_index

# Other trajectory IDs that also use this exact virtual-net edge.
string[] conflicting_trajectory_ids
