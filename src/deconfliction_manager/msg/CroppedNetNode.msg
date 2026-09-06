# One non-collision virtual-net node retained by the crop.
# The physical node is emitted once even when partial repetitions visit it
# several times.

uint64 node_id
geometry_msgs/Point position

# Phases in which this node occurs.
# 0=TAKEOFF, 1=MISSION, 2=LANDING.
uint8[] phases

# Original mission[] components in which this node occurs.
# TAKEOFF/LANDING do not add a mission index here.
uint32[] mission_indices
