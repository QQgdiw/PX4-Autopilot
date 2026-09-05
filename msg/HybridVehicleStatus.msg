uint64 timestamp          # time since system start (microseconds)
uint8 HYBRID_STATE_FLYING = 0
uint8 HYBRID_STATE_TRANSITIONING = 1
uint8 HYBRID_STATE_DRIVING = 2
uint8 HYBRID_STATE_UNKNOWN = 3
uint8 HYBRID_STATE_TRANSITION_FAULT = 4

uint8 TARGET_NONE = 0
uint8 TARGET_FLYING = 1
uint8 TARGET_DRIVING = 2

uint8 SENSOR_NONE = 0
uint8 SENSOR_AS5600 = 1
uint8 SENSOR_TMAG5273 = 2
uint8 SENSOR_HX8 = 3
uint8 SENSOR_HX65 = 4

uint8 ACTUATOR_PWM = 0
uint8 ACTUATOR_HX8 = 1
uint8 ACTUATOR_HX65 = 2

uint8 PROPULSION_NONE = 0
uint8 PROPULSION_QUAD = 1
uint8 PROPULSION_ROVER = 2

uint8 SEQUENCE_STABLE_QUAD = 0
uint8 SEQUENCE_Q2R_PREPARE = 1
uint8 SEQUENCE_Q2R_TRANSFORM = 2
uint8 SEQUENCE_ROVER_RETRACT = 3
uint8 SEQUENCE_STABLE_ROVER = 4
uint8 SEQUENCE_R2Q_PREPARE = 5
uint8 SEQUENCE_R2Q_TRANSFORM = 6
uint8 SEQUENCE_QUAD_WAIT_AIRBORNE = 7
uint8 SEQUENCE_QUAD_RETRACT = 8
uint8 SEQUENCE_FAULT = 9

uint8 SEQUENCE_FAULT_NONE = 0
uint8 SEQUENCE_FAULT_GEAR_COMMUNICATION = 1
uint8 SEQUENCE_FAULT_GEAR_TIMEOUT = 2
uint8 SEQUENCE_FAULT_SHAPE = 3
uint8 SEQUENCE_FAULT_INVALID_CONFIGURATION = 4

uint8 TRANSFORM_FAULT_NONE = 0
uint8 TRANSFORM_FAULT_NO_SENSOR = 1
uint8 TRANSFORM_FAULT_SENSOR_CONFLICT = 2
uint8 TRANSFORM_FAULT_SENSOR_TIMEOUT = 3
uint8 TRANSFORM_FAULT_TRANSITION_TIMEOUT = 4
uint8 TRANSFORM_FAULT_INVALID_SERVO_CONFIG = 5
uint8 TRANSFORM_FAULT_INVALID_CONFIGURATION = 6
uint8 TRANSFORM_FAULT_STALL = 7
uint8 TRANSFORM_FAULT_ACTUATOR_COMMUNICATION = 8
uint8 TRANSFORM_FAULT_ACTUATOR_PROTECTION = 9
uint8 TRANSFORM_FAULT_ACTUATOR_CONFIG_MISMATCH = 10
uint8 TRANSFORM_FAULT_ACTUATOR_COMMAND_REJECTED = 11

uint8 current_state       # 当前的变形状态
uint8 target_state        # requested stable state during a transition
uint8 sensor_source       # source currently confirming the mechanism position
uint8 fault_reason        # transformation fault code, zero when healthy
bool as5600_valid
bool tmag_quad_valid
bool tmag_rover_valid
bool sensors_enabled      # sensor-backed shape validation is configured
bool position_confirmed   # current stable endpoint is confirmed by a fresh source
uint64 transition_elapsed # elapsed transition time (microseconds)
uint8 actuator_backend
float32 position_normalized
bool position_valid
bool actuator_online
bool actuator_healthy
bool actuator_config_verified
uint8 actuator_protection_flags
uint64 no_progress_elapsed

uint8 COMMAND_RESULT_NONE = 255
uint8 REJECT_NONE = 0
uint8 REJECT_INVALID_TARGET = 1
uint8 REJECT_RESERVED_PARAMETER = 2
uint8 REJECT_NOT_LANDED = 3
uint8 REJECT_LAND_DETECTOR_STALE = 4
uint8 REJECT_OPPOSITE_TRANSITION = 5
uint8 REJECT_TRANSFORMATION_FAULT = 6
uint8 REJECT_UNKNOWN_STATE = 7

uint32 transition_sequence
uint64 transition_completed_timestamp
uint8 command_result
uint8 command_reject_reason
bool landed
bool land_detection_fresh
uint64 command_timestamp # timestamp of the vehicle_command whose result is reported
uint8 sequence_state
uint8 sequence_fault
uint8 propulsion_owner
bool propulsion_ready
bool gear_online
bool gear_healthy
bool gear_down
bool gear_clear
bool gear_stowed
float32 gear_angle_deg
