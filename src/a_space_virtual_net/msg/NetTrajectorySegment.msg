# One static-trajectory portion projected onto exactly one edge of the A-space net.

uint8 TAKEOFF=0
uint8 MISSION=1
uint8 LANDING=2

uint8 AXIS_X=0
uint8 AXIS_Y=1
uint8 AXIS_Z=2

uint64 edge_id
uint8 axis

uint64 start_node_id
uint64 end_node_id

geometry_msgs/Point start
geometry_msgs/Point end
float64 length_m

# Phase and phase-local source-segment index in the expanded operation.
#
# With the multi-mission StaticTrajectory interface, all mission components
# share the MISSION phase. The index is monotonically increasing across:
#
#   repetition 0: mission[0], mission[1], ...
#   repetition 1: mission[0], mission[1], ...
#
# Consecutive indices DO NOT imply geometric continuity between independent
# mission components.
uint8 phase
uint32 original_phase_segment_index

# Order of this lattice edge inside the projected output stream.
# Consecutive net_segment_index values also do NOT imply that two edges are
# geometrically connected; start_node_id/end_node_id define connectivity.
uint32 net_segment_index
