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
 * Transformation Duration (Fallback)
 *
 * The physical time previously required for open-loop transformation.
 * In closed-loop mode with encoders, this serves as a fallback timeout.
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
