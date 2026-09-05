# One virtual-net node retained by the cropped supervision geometry.
# Nodes are geometry-only and therefore appear once even if partial mission
# repetitions visit the same lattice node more than once.

uint64 node_id
geometry_msgs/Point position

# Phases in which this node was used by the original net-loaded trajectory.
# 0=TAKEOFF, 1=MISSION, 2=LANDING.
uint8[] phases
