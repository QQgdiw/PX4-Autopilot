uint64 timestamp
uint64 left_last_valid_response
uint64 right_last_valid_response
uint32 command_sequence
uint32 timeout_count
uint32 retry_count
uint32 protocol_error_count

uint8 RESULT_NONE = 0
uint8 RESULT_ACCEPTED = 1
uint8 RESULT_REJECTED = 2
uint8 RESULT_TIMEOUT = 3
uint8 RESULT_PROTOCOL_ERROR = 4

uint8 command_result
uint8 left_id
uint8 right_id
bool left_online
bool right_online
bool left_healthy
bool right_healthy
bool left_config_verified
bool right_config_verified
bool motion_config_valid
bool left_position_valid
bool right_position_valid
bool left_moving
bool right_moving
int16 left_position_steps
int16 right_position_steps
int16 left_speed_steps_s
int16 right_speed_steps_s
int16 left_load
int16 right_load
uint16 left_current_ma
uint16 right_current_ma
float32 left_voltage_v
float32 right_voltage_v
float32 left_temperature_c
float32 right_temperature_c
uint8 left_error_flags
uint8 right_error_flags
