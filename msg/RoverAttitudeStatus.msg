uint64 timestamp # time since system start (microseconds)

float32 measured_yaw          # [rad] Measured yaw
float32 adjusted_yaw_setpoint # [rad] Yaw setpoint that is being tracked (Applied slew rates)

bool active # True when the yaw attitude controller ran for this sample
