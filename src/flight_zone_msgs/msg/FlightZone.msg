std_msgs/Header header

uint8 INCLUSION=0
uint8 EXCLUSION=1
uint8 WARNING=2

string zone_id
string vehicle_id
string display_name
uint8 zone_type

# Polyhedron geometry expressed in header.frame_id.
geometry_msgs/Point[] vertices
flight_zone_msgs/PolyhedronFace[] faces

# Reserved for later conflict/geofencing calculations.
float64 safety_margin
bool enabled
