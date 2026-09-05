uint64 timestamp
uint32 sequence

uint8 COMMAND_NONE = 0
uint8 COMMAND_MOVE = 1
uint8 COMMAND_RELEASE = 2
uint8 COMMAND_HOLD = 3
uint8 COMMAND_GEAR_MOVE = 4
uint8 COMMAND_GEAR_HOLD = 5

uint8 type
uint8 servo_id
float32 target_angle_deg
uint16 move_time_ms
uint16 acceleration_time_ms
uint16 deceleration_time_ms
uint16 power_mw
