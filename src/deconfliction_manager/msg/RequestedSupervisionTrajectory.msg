# One trajectory selected for automatic supervision/deconfliction.
#
# detected_collision preserves the complete original collision evidence.
# cropped_trajectory is the authoritative crop generated in this node and now
# contains a ready-to-use multi-mission StaticTrajectory.

collision_detection/DetectedCollisionTrajectory detected_collision
deconfliction_manager/CroppedNetTrajectory cropped_trajectory
