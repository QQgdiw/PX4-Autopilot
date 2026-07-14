uint64 timestamp                       # time since system start (microseconds)

uint32 DRIVE_FAULT_NONE = 0
uint32 DRIVE_FAULT_LEFT_FEEDBACK = 1
uint32 DRIVE_FAULT_RIGHT_FEEDBACK = 2
uint32 DRIVE_FAULT_CAN = 4
uint32 DRIVE_FAULT_COMMAND = 8

# Array index 0 is the left motor (C610 ID 1).
# Array index 1 is the right motor (C610 ID 2).
uint64[2] feedback_timestamp           # last valid feedback time (microseconds)
float32[2] target_rpm                  # output-shaft speed setpoint
float32[2] measured_rpm                # measured output-shaft speed
float32[2] speed_error                 # target minus measured speed
int16[2] current_command               # commanded C610 torque current
int16[2] torque_current                # measured C610 torque current
uint16[2] encoder                      # raw C610 encoder position
bool[2] online                         # feedback is fresh

uint32 fault_flags
uint32 rx_count
uint32 tx_count
uint32 timeout_count
uint64 can_error_count
