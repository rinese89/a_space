# One trajectory selected for automatic supervision/deconfliction.
#
# detected_collision preserves the complete previous
# collision_detection/DetectedCollisionTrajectory payload.
# cropped_trajectory adds the geometry-only virtual-net crop.

collision_detection/DetectedCollisionTrajectory detected_collision
deconfliction_manager/CroppedNetTrajectory cropped_trajectory
