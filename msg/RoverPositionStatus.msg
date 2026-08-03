uint64 timestamp # time since system start (microseconds)

float32[2] position_ned # Current position in the local NED frame [m]
float32[2] target_ned   # Active path target in the local NED frame [m]

float32 crosstrack_error     # Signed shortest distance to the active path [m]
float32 lookahead_distance   # Pure Pursuit lookahead distance [m]
float32 target_bearing       # Pure Pursuit target bearing [rad]
float32 distance_to_waypoint # Distance to the active waypoint [m]

uint8 xy_reset_counter # Index of the latest local XY position reset

bool active         # True when the position controller is active and configured
bool position_valid # True when position_ned is finite and valid
bool target_valid   # True when target_ned is the active path target
bool path_valid     # True when the Pure Pursuit diagnostic fields are valid
