# Complete static operation: takeoff -> mission -> landing.
# Lower numerical priority means higher priority.

int32 priority
string trajectory_id

uint32 ua_id
string uas_namespace
string flight_zone_id
string frame_id
string action_name

static_trajectory_manager/TrajectorySegment takeoff
static_trajectory_manager/TrajectorySegment mission
static_trajectory_manager/TrajectorySegment landing

float64 goal_tolerance
float64 slowdown_radius

# Partial repetition: number of mission executions inside ONE complete operation.
uint32 repetitions

# Period, in seconds, between starts of consecutive complete operations.
# 0.0 disables total repetition. The static trajectory manager materialises
# one concrete complete occurrence at a time and advances its start/end after
# receiving the completed occurrence through /adjusted_trajectories.
float64 operation_frequency

float64 average_speed_mps
float64 estimated_distance_m
float64 estimated_duration_s

builtin_interfaces/Time operation_start_utc
builtin_interfaces/Time operation_end_utc

# Signed real-vs-planned end-time difference for a completed occurrence.
# It may be positive, zero or negative. The static trajectory manager absorbs
# it into operation_end_utc and republishes future occurrences with 0.0.
float64 extra_time

# Preserved compatibility fields used by the trajectory-management chain.
bool spacial_conflict
string spacial_conflicting_trajectory_id
