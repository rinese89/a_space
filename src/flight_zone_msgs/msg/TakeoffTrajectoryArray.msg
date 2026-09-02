# Takeoff trajectory set.
#
# /takeoff_waypoints: static/geometric candidates from controlled_invasion_supervisor_node.
# /checked_takeoff_waypoints: dynamically authorized candidates from dynamic_supervisor_node.

std_msgs/Header header

float64 minimum_separation_m
bool collision_data_valid
uint32 total_count

flight_zone_msgs/TakeoffTrajectory[] trajectories

string[] rejected_trajectory_ids
string[] rejection_reasons
