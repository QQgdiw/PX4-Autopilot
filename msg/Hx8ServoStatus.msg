uint64 timestamp
uint64 timestamp_sample
uint64 last_valid_response
uint32 command_sequence
uint32 rx_valid_count
uint32 rx_error_count
uint32 timeout_count
uint32 retry_count
uint8 servo_id

uint8 RESULT_NONE = 0
uint8 RESULT_ACCEPTED = 1
uint8 RESULT_REJECTED = 2
uint8 RESULT_TIMEOUT = 3
uint8 RESULT_PROTOCOL_ERROR = 4

uint8 STATUS_COMMAND_EXECUTING = 1
uint8 STATUS_COMMAND_ERROR = 2
uint8 PROTECTION_STALL = 4
uint8 PROTECTION_HIGH_VOLTAGE = 8
uint8 PROTECTION_LOW_VOLTAGE = 16
uint8 PROTECTION_OVERCURRENT = 32
uint8 PROTECTION_POWER = 64
uint8 PROTECTION_TEMPERATURE = 128

bool online
bool healthy
bool config_verified
bool command_accepted
bool persistent_write_active
float32 angle_deg
float32 voltage_v
float32 current_a
float32 power_w
float32 temperature_c # Celsius converted from the vendor appendix ADC table
uint8 status_flags     # complete raw protocol status byte
uint8 protection_flags # status_flags & 0xfc; excludes command status bits
uint8 command_result
