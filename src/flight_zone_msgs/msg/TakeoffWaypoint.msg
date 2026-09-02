# Waypoint de despegue comun para los UAS de una flight zone.
# Publicado en /<flight_zone_id>/takeoff_waypoints.
#
# x/y/z/yaw conservan la semantica anterior: coordenadas en el odom
# local de cada UAS de la zona. header.frame_id es normalmente "odom".

std_msgs/Header header
string flight_zone_id
float64 x
float64 y
float64 z
float64 yaw
