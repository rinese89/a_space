# Authoritative snapshot of trajectories classified as COLLISION by the first
# stage, enriched with exact shared virtual-net nodes and edges.
#
# Each contained trajectory preserves the complete multi-mission
# static_trajectory_manager/StaticTrajectory.

std_msgs/Header header
collision_detection/DetectedCollisionTrajectory[] trajectories
