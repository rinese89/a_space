# Authoritative state of the complete A-space Cartesian virtual net.

std_msgs/Header header
geometry_msgs/Point origin
float64 size_x
float64 size_y
float64 size_z
float64 density_net
uint32 nodes_x
uint32 nodes_y
uint32 nodes_z
uint64 total_edges
uint64 free_edges
uint64 available_edges
uint64 collision_clear_edges
uint64 collision_edges
uint64 total_capacity_units
uint64 occupied_capacity_units
float64 global_utilization
a_space_virtual_net/VirtualNetEdge[] edges
