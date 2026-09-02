std_msgs/Header header

uint8 NO_ZONE=0
uint8 INSIDE=1
uint8 OUTSIDE=2
uint8 ON_BOUNDARY=3
uint8 EXCLUSION_VIOLATION=4
uint8 TRANSFORM_ERROR=5
uint8 INVALID_GEOMETRY=6

string vehicle_id
uint8 state

# True only when the vehicle is accepted inside an inclusion volume and is not
# violating an exclusion volume. Boundary handling is configurable in the node.
bool inside_allowed_volume
bool inside_exclusion_zone

# Vehicle position transformed into header.frame_id, normally map.
geometry_msgs/Point position
string source_frame

flight_zone_msgs/ZoneContainment[] zones
string detail
