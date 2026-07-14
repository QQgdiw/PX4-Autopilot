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

uint8 TRANSFORM_FAULT_NONE = 0
uint8 TRANSFORM_FAULT_NO_SENSOR = 1
uint8 TRANSFORM_FAULT_SENSOR_CONFLICT = 2
uint8 TRANSFORM_FAULT_SENSOR_TIMEOUT = 3
uint8 TRANSFORM_FAULT_TRANSITION_TIMEOUT = 4
uint8 TRANSFORM_FAULT_INVALID_SERVO_CONFIG = 5

uint8 current_state       # 当前的变形状态
uint8 target_state        # requested stable state during a transition
uint8 sensor_source       # source currently confirming the mechanism position
uint8 fault_reason        # transformation fault code, zero when healthy
bool as5600_valid
bool tmag_quad_valid
bool tmag_rover_valid
uint64 transition_elapsed # elapsed transition time (microseconds)
