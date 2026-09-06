# One retained/removed virtual-net edge annotated with the original
# multi-mission component that produced it.
#
# NO_MISSION is used for TAKEOFF and LANDING.
uint32 NO_MISSION=4294967295

uint32 mission_index

# Partial mission repetition that produced the first emitted geometric copy.
# Repeated copies of the same (phase, mission_index, edge_id) are collapsed.
uint32 source_repetition

a_space_virtual_net/NetTrajectorySegment segment
