/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file hybrid_vehicle_control_params.c
 *
 * Quad-Rover 混合载具变形模块的参数定义。
 * 包含状态机阈值、遥控器通道映射以及闭环传感器的目标角度。
 */

/**
 * Maximum Transformation Duration
 *
 * With position sensors enabled, expiry is a transition fault. With sensors
 * disabled, expiry completes the open-loop transition.
 *
 * @group Hybrid Control
 * @unit s
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(HYBRID_TRANS_T, 6.0f);

/**
 * Enable transformation position sensors
 *
 * Enables AS5600 and TMAG5273 endpoint completion checks. Runtime sensor
 * faults do not fall back to open-loop operation.
 *
 * @boolean
 * @group Hybrid Control
 */
PARAM_DEFINE_INT32(HYB_SENS_EN, 1);

/**
 * Configured startup shape
 *
 * This persistent operator configuration is used as the startup assumption
 * only when position sensors are disabled. Runtime transitions do not update it.
 *
 * @value 0 Quad
 * @value 1 Rover
 * @group Hybrid Control
 */
PARAM_DEFINE_INT32(HYB_BOOT_ST, 0);

/**
 * Quad shape Servo 1 target
 *
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @group Hybrid Control
 */
PARAM_DEFINE_FLOAT(HYB_SV_QUD, 0.0f);

/**
 * Rover shape Servo 1 target
 *
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @group Hybrid Control
 */
PARAM_DEFINE_FLOAT(HYB_SV_ROV, 0.0f);

/**
 * AS5600 endpoint angle tolerance
 *
 * @unit rad
 * @min 0.0
 * @max 3.14
 * @decimal 3
 * @group Hybrid Control
 */
PARAM_DEFINE_FLOAT(HYB_ANG_TOL, 0.05f);

/**
 * Transformation position feedback timeout
 *
 * @unit s
 * @min 0.01
 * @max 5.0
 * @decimal 2
 * @group Hybrid Control
 */
PARAM_DEFINE_FLOAT(HYB_SENS_TO, 0.30f);

/**
 * Endpoint confirmation time
 *
 * Position feedback must continuously confirm an endpoint for this duration.
 *
 * @unit s
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Hybrid Control
 */
PARAM_DEFINE_FLOAT(HYB_DBNC_T, 0.10f);

/**
 * Maximum Transformation Altitude
 *
 * The maximum allowable altitude (above ground/local frame) to permit a transition
 * from flying (multicopter) to driving (rover) mode.
 *
 * @group Hybrid Control
 * @unit m
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(HYBRID_MAX_Z, 0.5f);

/**
 * Manual Override RC Switch Channel
 *
 * The auxiliary RC channel used to directly take over the transformation mechanism.
 * When this switch is toggled, it overrides the auto state machine and maps
 * the channel value directly to the mechanism servo outputs.
 *
 * @group Hybrid Control
 * @min 1
 * @max 6
 * @reboot_required true
 */
PARAM_DEFINE_INT32(HYBRID_MAN_CH, 4);

/**
 * Target Mechanism Angle (Rover Mode)
 *
 * The target angle read from the absolute encoder (e.g., AS5600) when the
 * vehicle is fully transformed into Rover (Driving) mode.
 *
 * @group Hybrid Control
 * @unit rad
 * @min 0
 * @max 6.28
 * @decimal 2
 * @increment 0.05
 */
PARAM_DEFINE_FLOAT(HYBRID_ANG_ROV, 3.14f);

/**
 * Target Mechanism Angle (Quad Mode)
 *
 * The target angle read from the absolute encoder (e.g., AS5600) when the
 * vehicle is fully transformed into Quadcopter (Flying) mode.
 *
 * @group Hybrid Control
 * @unit rad
 * @min 0
 * @max 6.28
 * @decimal 2
 * @increment 0.05
 */
PARAM_DEFINE_FLOAT(HYBRID_ANG_QUD, 1.57f);

/**
 * Quad Mode Magnetic Sensor Device ID
 *
 * The unique device ID of the TMAG5273 sensor installed at the Quadcopter limit.
 * (Check the device_id using `listener magnetic_sensor` in nsh).
 *
 * @group Hybrid Control
 */
PARAM_DEFINE_INT32(HYB_MAG_ID_QUD, 53);//0x35

/**
 * Rover Mode Magnetic Sensor Device ID
 *
 * The unique device ID of the TMAG5273 sensor installed at the Rover limit.
 *
 * @group Hybrid Control
 */
PARAM_DEFINE_INT32(HYB_MAG_ID_ROV, 34);//0x22

/**
 * Target Magnetic Field Threshold (Quad Mode)
 *
 * The absolute Z-axis magnetic field strength (in mT) required to confirm Quad mode transformation.
 *
 * @group Hybrid Control
 * @min 0.0
 * @max 100.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(HYB_MAG_THR_QUD, 5.0f);

/**
 * Target Magnetic Field Threshold (Rover Mode)
 *
 * The absolute Z-axis magnetic field strength (in mT) required to confirm Rover mode transformation.
 *
 * @group Hybrid Control
 * @min 0.0
 * @max 100.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(HYB_MAG_THR_ROV, 5.0f);
