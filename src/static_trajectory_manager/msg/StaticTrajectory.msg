int32 priority
string trajectory_id

uint32 ua_id
string uas_namespace
string flight_zone_id
string frame_id
string action_name

# TAKEOFF and LANDING remain single continuous polylines.
static_trajectory_manager/TrajectorySegment takeoff

# Ordered collection of independent mission polylines.
# IMPORTANT: no geometric edge is implied between mission[i] and mission[i+1].
# This permits a supervised cropped trajectory to preserve disconnected
# collision-free pieces without drawing an artificial bridge across a removed
# collision section.
static_trajectory_manager/TrajectorySegment[] mission

static_trajectory_manager/TrajectorySegment landing

float64 goal_tolerance
float64 slowdown_radius

# Partial repetition of the mission collection within one total operation.
uint32 repetitions

# Period in seconds between complete-operation starts.
# 0.0 disables total repetition.
float64 operation_frequency

float64 average_speed_mps
float64 estimated_distance_m
float64 estimated_duration_s

builtin_interfaces/Time operation_start_utc
builtin_interfaces/Time operation_end_utc

float64 extra_time

# Existing field names intentionally preserved for compatibility.
bool spacial_conflict
string spacial_conflicting_trajectory_id
