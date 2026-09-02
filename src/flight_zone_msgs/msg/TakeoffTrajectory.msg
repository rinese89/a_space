# Takeoff trajectory expressed in header.frame_id, normally map.
# Currently waypoints[0] is spawn and waypoints[1] is the takeoff target.
# All conflict arrays are aligned by index.

std_msgs/Header header

string trajectory_id
string flight_zone_id
string uas_namespace

geometry_msgs/Point[] waypoints
float64 yaw

bool geometric_conflict

string[] conflict_sources
string[] conflicting_ids
uint32[] conflicting_ua_ids
string[] conflicting_uas_namespaces
string[] conflicting_flight_zone_ids
float64[] conflict_distances_m
