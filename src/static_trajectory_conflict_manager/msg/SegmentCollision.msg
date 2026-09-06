uint8 TAKEOFF=0
uint8 MISSION=1
uint8 LANDING=2

uint8 own_phase
uint32 own_segment_index
geometry_msgs/Point own_closest_point

uint8 conflicting_phase
uint32 conflicting_segment_index
geometry_msgs/Point conflicting_closest_point

float64 minimum_distance_m
