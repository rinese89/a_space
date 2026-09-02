# Persistent state of one Cartesian lattice edge.

uint8 AXIS_X=0
uint8 AXIS_Y=1
uint8 AXIS_Z=2

uint8 FREE=0
uint8 AVAILABLE=1
uint8 COLLISION_CLEAR=2
uint8 COLLISION=3

uint64 edge_id
uint8 axis
geometry_msgs/Point start
geometry_msgs/Point end
uint8 state
uint32 capacity
uint32 occupancy
float64 utilization
string[] available_trajectory_ids
string[] collision_clear_trajectory_ids
string[] collision_trajectory_ids
# Kept for compatibility. This revision no longer subscribes to /flight_zones.
string[] flight_zone_ids
