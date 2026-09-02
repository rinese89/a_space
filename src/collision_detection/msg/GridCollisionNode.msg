# One virtual-net node occupied by the target collision trajectory and at least
# one other loaded trajectory.

uint64 node_id
geometry_msgs/Point position

# Net-segment indices of the target trajectory that touch this node.
uint32[] own_net_segment_indices

# Other trajectory IDs that also use this node.
string[] conflicting_trajectory_ids
