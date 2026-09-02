std_msgs/Header header

# Stable identifier formed from the two incompatible zones.
string conflict_id

string vehicle_id_a
string display_name_a
string zone_id_a

string vehicle_id_b
string display_name_b
string zone_id_b

# Margins actually applied to each zone in XY and Z [m].
float64 safety_margin_a
float64 safety_margin_b

# Horizontal intersection of the safety-expanded footprints [m^2].
float64 horizontal_overlap_area
bool horizontal_boundary_contact_only

# Vertical intersection of the safety-expanded altitude intervals [m].
float64 vertical_overlap
float64 overlap_min_z
float64 overlap_max_z
bool vertical_boundary_contact_only

# Footprints resulting from the horizontal intersection, expressed in map.
# The z component of each point is set to overlap_min_z.
geometry_msgs/Polygon[] horizontal_intersections
