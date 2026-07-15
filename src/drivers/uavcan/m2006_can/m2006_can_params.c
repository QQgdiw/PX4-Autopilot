/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * Enable M2006 CAN wheel driver
 *
 * @boolean
 * @reboot_required true
 * @group M2006 CAN
 */
PARAM_DEFINE_INT32(M2K_EN, 1);

/**
 * Left motor C610 ID
 *
 * This hardware integration supports only left ID 1 and right ID 2.
 *
 * @min 1
 * @max 1
 * @reboot_required true
 * @group M2006 CAN
 */
PARAM_DEFINE_INT32(M2K_L_ID, 1);

/**
 * Right motor C610 ID
 *
 * This hardware integration supports only left ID 1 and right ID 2.
 *
 * @min 2
 * @max 2
 * @reboot_required true
 * @group M2006 CAN
 */
PARAM_DEFINE_INT32(M2K_R_ID, 2);

/**
 * Reverse left motor direction
 *
 * @boolean
 * @group M2006 CAN
 */
PARAM_DEFINE_INT32(M2K_L_REV, 0);

/**
 * Reverse right motor direction
 *
 * @boolean
 * @group M2006 CAN
 */
PARAM_DEFINE_INT32(M2K_R_REV, 0);

/**
 * Maximum motor output-shaft speed
 *
 * @unit rpm
 * @min 1.0
 * @max 500.0
 * @decimal 1
 * @group M2006 CAN
 */
PARAM_DEFINE_FLOAT(M2K_MAX_RPM, 500.0f);

/**
 * C610 current command limit
 *
 * @min 0
 * @max 10000
 * @group M2006 CAN
 */
PARAM_DEFINE_INT32(M2K_CUR_LIM, 10000);

/**
 * Speed controller proportional gain
 *
 * @min 0.0
 * @decimal 4
 * @group M2006 CAN
 */
PARAM_DEFINE_FLOAT(M2K_SPD_P, 0.0f);

/**
 * Speed controller integral gain
 *
 * @min 0.0
 * @decimal 4
 * @group M2006 CAN
 */
PARAM_DEFINE_FLOAT(M2K_SPD_I, 0.0f);

/**
 * Speed controller derivative gain
 *
 * @min 0.0
 * @decimal 4
 * @group M2006 CAN
 */
PARAM_DEFINE_FLOAT(M2K_SPD_D, 0.0f);

/**
 * Speed controller feedforward gain
 *
 * @min 0.0
 * @decimal 4
 * @group M2006 CAN
 */
PARAM_DEFINE_FLOAT(M2K_SPD_FF, 0.0f);

/**
 * Motor target speed slew rate
 *
 * The value is expressed in rpm per second.
 *
 * @unit rpm
 * @min 0.0
 * @decimal 1
 * @group M2006 CAN
 */
PARAM_DEFINE_FLOAT(M2K_RPM_SLEW, 500.0f);

/**
 * Motor feedback timeout
 *
 * @unit s
 * @min 0.001
 * @max 1.0
 * @decimal 3
 * @group M2006 CAN
 */
PARAM_DEFINE_FLOAT(M2K_FB_TO, 0.05f);

/**
 * Final actuator command timeout
 *
 * @unit s
 * @min 0.001
 * @max 1.0
 * @decimal 3
 * @group M2006 CAN
 */
PARAM_DEFINE_FLOAT(M2K_CMD_TO, 0.10f);
