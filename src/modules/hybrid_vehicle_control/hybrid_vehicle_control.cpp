/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "hybrid_vehicle_control.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/events.h>

using namespace time_literals;

// ==============================================================================
// 构造函数与析构函数
// ==============================================================================

HybridVehicleControl::HybridVehicleControl() :
	ModuleParams(nullptr),
	// 将此模块挂载到 nav_and_controllers (导航与控制器) 工作队列中
	// 这与 vtol_att_control 和 mc_pos_control 运行在同一个线程池，保证数据同步
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	// 初始化时强制更新一次参数
	updateParams();
}

HybridVehicleControl::~HybridVehicleControl()
{
	// 模块退出时的清理工作（如果有的话）
}

// ==============================================================================
// 初始化与调度
// ==============================================================================

bool HybridVehicleControl::init()
{
	// 设定主循环的运行频率。
	// 对于状态机和模式仲裁，50Hz (20ms) 是一个非常平衡且标准的频率。
	// 具体的姿态控制由底层的 mc_att_control (高频，如 400Hz) 负责，我们不需要那么快。
	ScheduleOnInterval(20_ms);

	return true;
}

// ==============================================================================
// PX4 标准命令行接口 (CLI)
// ==============================================================================

int HybridVehicleControl::task_spawn(int argc, char *argv[])
{
	// 实例化模块对象
	HybridVehicleControl *instance = new HybridVehicleControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

		// 如果初始化失败，清理内存
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int HybridVehicleControl::custom_command(int argc, char *argv[])
{
	// 如果你以后想在终端输入 "hybrid_vehicle_control force_rover" 之类的自定义命令，
	// 可以写在这个函数里。目前返回未识别。
	return print_usage("unrecognized command");
}

int HybridVehicleControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Hybrid vehicle control module (Quad-Rover).
Handles the state machine and control allocation transition between
multicopter and rover modes.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hybrid_vehicle_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

// ==============================================================================
// 系统入口函数
// ==============================================================================

extern "C" __EXPORT int hybrid_vehicle_control_main(int argc, char *argv[])
{
	// 直接调用 ModuleBase 提供的默认主函数逻辑
	return HybridVehicleControl::main(argc, argv);
}

// ==============================================================================
// 核心工作循环 (运行在 Work Queue 中)
// ==============================================================================

void HybridVehicleControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	updateParams();

	vehicle_status_s vehicle_status;
	if (_vehicle_status_sub.update(&vehicle_status)) {
		_is_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	}

	// 仅监听当前的模式状态，绝不覆写
	_vehicle_control_mode_sub.update(&_vcontrol_mode);

	// 核心：执行状态机与模式切换逻辑 (遥控器与航线解析)
	update_state_machine();

	// ====================================================================
	// [全新逻辑] 广播当前的物理形态，让 Commander 决定如何分配动力
	// ====================================================================
	hybrid_vehicle_status_s status_msg{};
	status_msg.timestamp = hrt_absolute_time();

	if (_current_state == HybridState::FLYING) {
		status_msg.current_state = hybrid_vehicle_status_s::HYBRID_STATE_FLYING;
	} else if (_current_state == HybridState::DRIVING) {
		status_msg.current_state = hybrid_vehicle_status_s::HYBRID_STATE_DRIVING;
	} else {
		// 正在变车或变飞机，都属于过渡态
		status_msg.current_state = hybrid_vehicle_status_s::HYBRID_STATE_TRANSITIONING;
	}
	_hybrid_status_pub.publish(status_msg);
	// ====================================================================

	control_transformation_actuators();

	// ====================================================================
	// [终极硬件多路复用器] 数据搬运与安全隔离
	// ====================================================================

	// 1. 从两个旁路拉取最新数据 (由原生的 Allocator 和 Rover 模块计算得出)
	_actuator_motors_mc_sub.update(&_mc_motors);
	_actuator_motors_rover_sub.update(&_rover_motors);

	// 2. 准备发往底层物理针脚的空载体
	actuator_motors_s final_motors{};
	final_motors.timestamp = hrt_absolute_time();
	final_motors.timestamp_sample = final_motors.timestamp;

	// 【安全第一】默认将所有通道设为 NAN (物理输出将被锁定在 Disarmed 安全低电平)
	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		final_motors.control[i] = NAN;
	}

	// 映射表：根据你的 QGC MAIN 针脚分配，锁定车轮的数组索引
	const int ROVER_LEFT_IDX = 4;  // MAIN 5
	const int ROVER_RIGHT_IDX = 5; // MAIN 6

	// 3. 根据系统当前形态，进行物理通道的“道岔”切换
	if (_current_state == HybridState::FLYING) {
		// --- 飞行模式 ---
		// 仅接通多旋翼的动力 (前 4 个通道)
		for (int i = 0; i < 4; i++) {
			final_motors.control[i] = _mc_motors.control[i];
		}
		final_motors.reversible_flags = _mc_motors.reversible_flags;
		// 此时车轮通道保持 NAN，在空中绝对静止

	} else if (_current_state == HybridState::DRIVING) {
		// --- 漫游车模式 ---
		// 仅接通车轮的动力
		// 将原生 rover_differential 算好的左右轮转速，搬运到指定的物理通道
		final_motors.control[ROVER_LEFT_IDX] = _rover_motors.control[0];
		final_motors.control[ROVER_RIGHT_IDX] = _rover_motors.control[1];
		final_motors.reversible_flags = _rover_motors.reversible_flags;
		// 此时螺旋桨通道保持 NAN，在地上绝对锁死，防止削人

	} else {
		// --- 变形过渡态 (TRANSITIONING) ---
		// 最危险的物理状态。不执行任何搬运，所有动力通道保持 NAN。
		// 全车的无刷动力瞬间切断，仅由 actuator_servos 驱动变形舵机。
	}

	// 4. 终极发布：将绝对安全的物理指令推向底层 PWM/DShot 驱动
	_actuator_motors_final_pub.publish(final_motors);
}

// ==============================================================================
// 状态机逻辑
// ==============================================================================

void HybridVehicleControl::update_state_machine()
{
	bool request_rover = false;
	bool request_quad  = false;

	// 获取当前是否在自动航线模式
	bool is_in_auto_mode = _vcontrol_mode.flag_control_auto_enabled;

	// =========================================================
	// 来源 1：解析 MAVLink 车辆指令 (航线节点 & QGC 滑块)
	// =========================================================
	vehicle_command_s vcmd{};
	while (_vehicle_command_sub.update(&vcmd)) {
		if (vcmd.command == vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION) {

			int transition_target = (int)(vcmd.param1 + 0.5f);
			// MAVLink 协议规定：3 = 切换到 MC (旋翼)，4 = 切换到 FW (我们的车)
			if (transition_target == 4) {
				request_rover = true;
				PX4_INFO("[Hybrid] Mission/QGC Cmd: Transition to ROVER");
			}
			else if (transition_target == 3) {
				request_quad = true;
				PX4_INFO("[Hybrid] Mission/QGC Cmd: Transition to QUAD");
			}

			// 极度关键：必须向 QGC 回复确认 (ACK)，否则任务节点会卡死报错
			vehicle_command_ack_s ack{};
			ack.timestamp = hrt_absolute_time();
			ack.command = vcmd.command;
			ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
			ack.target_system = vcmd.source_system;
			ack.target_component = vcmd.source_component;
			_vehicle_command_ack_pub.publish(ack);
		}
	}

	// =========================================================
	// 来源 2：解析遥控器拨杆 (跳变检测 + 主动降级)
	// =========================================================
	manual_control_setpoint_s manual_control;
	if (_manual_control_setpoint_sub.update(&manual_control)) {
		float current_transfer_switch = -1.0f; // 默认值
		// 将 QGC 里配置的 Channel 映射到 aux 变量
		switch (_rc_map_trans_sw_val) {
			case 5: current_transfer_switch = manual_control.aux1; break;
			case 6: current_transfer_switch = manual_control.aux2; break;
			case 7: current_transfer_switch = manual_control.aux3; break;
			case 8: current_transfer_switch = manual_control.aux4; break;
			case 9: current_transfer_switch = manual_control.aux5; break;
			case 10: current_transfer_switch = manual_control.aux6; break;
			default: break;
		}
		float current_manual_switch = -1.0f;
		switch (_param_hybrid_man_ch.get()) {
			case 1: current_manual_switch = manual_control.aux1; break;
			case 2: current_manual_switch = manual_control.aux2; break;
			case 3: current_manual_switch = manual_control.aux3; break;
			case 4: current_manual_switch = manual_control.aux4; break;
			case 5: current_manual_switch = manual_control.aux5; break;
			case 6: current_manual_switch = manual_control.aux6; break;
			default: current_manual_switch = manual_control.aux1; break;
		}

		_manual_rc_value = current_manual_switch; // 缓存当前的手动接管通道值，供状态机使用
		static float last_manual_switch = current_manual_switch;

		// --- 逻辑 A：手动接管通道发生跳变 ---
		// 阈值设为 0.5f 以过滤电位器轻微噪声
		if (fabsf(current_manual_switch - last_manual_switch) > 0.5f) {
			if (!_manual_override_active) {
				PX4_WARN("[Hybrid] MANUAL OVERRIDE ENGAGED!");
				_manual_override_active = true;
			}
		}

		// 检测到物理拨杆发生了跳变 (人工紧急介入)
		if (fabsf(current_transfer_switch - last_transfer_switch) > 0.5f) {
			PX4_INFO("[Hybrid] Main Mode Switch Changed!");

			// 核心：一旦主开关动作，立刻解除手动接管状态
			if (_manual_override_active) {
				PX4_INFO("[Hybrid] Manual Override Released.");
				_manual_override_active = false;
			}

			// 如果当前正在执行自动航线，立刻将其踢回 Position 模式
			if (is_in_auto_mode) {
				PX4_WARN("[Hybrid] Aborting Mission! Downgrading to Position Mode.");

				vehicle_command_s mode_cmd{};
				mode_cmd.timestamp = hrt_absolute_time();
				mode_cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
				// 参数 1：1 代表自定义模式 (Custom Mode)
				mode_cmd.param1 = 1.0f;
				// 参数 2：PX4 的 Main Mode 设为 3 (代表 Position 模式)
				mode_cmd.param2 = 3.0f; // PX4_CUSTOM_MAIN_MODE_POSCTL

				// 发布模式切换指令，系统 Commander 收到后会立刻接管并切断 Auto
				_vehicle_command_pub.publish(mode_cmd);
			}

			// 覆盖航线请求，执行遥控器指定的变形方向
			if (current_transfer_switch > 0.5f && last_transfer_switch <= 0.5f) {
				request_rover = true;
				request_quad = false;
				PX4_INFO("[Hybrid] RC Switch Flipped: Transition to ROVER");
			} else if (current_transfer_switch < -0.5f && last_transfer_switch >= -0.5f) {
				request_quad = true;
				request_rover = false;
				PX4_INFO("[Hybrid] RC Switch Flipped: Transition to QUAD");
			}
		}
		last_transfer_switch = current_transfer_switch;
		last_manual_switch = current_manual_switch;
	}

	hrt_abstime now = hrt_absolute_time();
	hrt_abstime transition_duration_us = (hrt_abstime)(_param_hybrid_trans_t.get() * 1000000.0f);

	// ---------------------------------------------------------
	// 执行状态机切换
	// ---------------------------------------------------------
	switch (_current_state) {
		case HybridState::FLYING:
			if (request_rover) {
				if (check_safe_to_transform(true)) {
					PX4_INFO("[Hybrid] Start transition: Flying -> Rover");
					_transition_start_time = now;
					_transformation_completed = false;
					_current_state = HybridState::TRANSITION_TO_ROVER;
				}
			}
			break;

		case HybridState::TRANSITION_TO_ROVER:
			if ((now - _transition_start_time > transition_duration_us) || _transformation_completed) {
				PX4_INFO("[Hybrid] Sensor Confirmed: DRIVING mode.");
				_current_state = HybridState::DRIVING;
			}
			break;

		case HybridState::DRIVING:
			if (request_quad) {
				PX4_INFO("[Hybrid] Start transition: Rover -> Flying");
				_transition_start_time = now;
				_transformation_completed = false;
				_current_state = HybridState::TRANSITION_TO_QUAD;
			}
			break;

		case HybridState::TRANSITION_TO_QUAD:
			if ((now - _transition_start_time > transition_duration_us) || _transformation_completed) {
				PX4_INFO("[Hybrid] Sensor Confirmed: FLYING mode.");
				_current_state = HybridState::FLYING;
			}
			break;
	}
}

// ==============================================================================
// 辅助控制与安全检查函数
// ==============================================================================

bool HybridVehicleControl::check_safe_to_transform(bool to_rover)
{
	if (to_rover) {
		vehicle_local_position_s local_pos{};
		if (_vehicle_local_position_sub.copy(&local_pos)) {
			// 在 PX4 的 NED 坐标系中，Z 轴向下为正。
			// 因此高度 (海拔之上) 实际上是负值。-local_pos.z 即为相对地面的正高度。
			float current_alt = -local_pos.z;

			// 如果当前高度大于允许的极限高度，则拒绝变形，防止高空断电摔机
			if (local_pos.z_valid && current_alt > _param_hybrid_max_z.get()) {
				return false;
			}
		}
	}
	return true;
}

void HybridVehicleControl::control_transformation_actuators()
{
    actuator_servos_s servos{};
    servos.timestamp = hrt_absolute_time();
    hrt_abstime now = servos.timestamp;

    // =======================================================
    // 1. 获取并分配传感器状态 (动态检测与 ID 路由)
    // =======================================================

    // 1.1 读取 AS5600 数据
    sensor_encoder_s encoder_data;
    if (_encoder_sub.update(&encoder_data)) {
        _current_mechanism_angle = encoder_data.position_rad;
        _last_encoder_timestamp = encoder_data.timestamp;
    }

    // 1.2 遍历读取所有 TMAG5273 数据，并根据 ID 参数路由给上/下限位
    for (int i = 0; i < _magnetic_subs.size(); i++) {
        magnetic_sensor_s mag_data;
        if (_magnetic_subs[i].update(&mag_data)) {
            // 通过比对地面站设定的 Device ID，确认是哪一个限位器的数据
            if (mag_data.device_id == (uint32_t)_param_hyb_mag_id_qud.get()) {
                _current_mag_z_qud = mag_data.mag_z;
                _last_mag_timestamp_qud = mag_data.timestamp;
            } else if (mag_data.device_id == (uint32_t)_param_hyb_mag_id_rov.get()) {
                _current_mag_z_rov = mag_data.mag_z;
                _last_mag_timestamp_rov = mag_data.timestamp;
            }
        }
    }

    // 1.3 判断哪些传感器是在线的 (300ms 超时判定)
    bool encoder_valid = (_last_encoder_timestamp != 0) && (now - _last_encoder_timestamp < 300_ms);
    bool mag_qud_valid = (_last_mag_timestamp_qud != 0) && (now - _last_mag_timestamp_qud < 300_ms);
    bool mag_rov_valid = (_last_mag_timestamp_rov != 0) && (now - _last_mag_timestamp_rov < 300_ms);

    // =======================================================
    // 2. 动力输出仲裁逻辑
    // =======================================================

    if (_manual_override_active) {
        // 【最高优先级】物理拨杆直接接管
        servos.control[0] = _manual_rc_value;
        servos.control[1] = -_manual_rc_value;
        _transformation_completed = true;
    }
    else {
        // 【自动闭环模式】仲裁机制：优先使用 AS5600 绝对编码器，如果没有则降级使用磁限位开关

        if (encoder_valid) {
            // ---------------------------------------------------
            // 模式 A: AS5600 连续角度闭环 (原有逻辑保留)
            // ---------------------------------------------------
            const float tolerance = 0.05f;

            if (_current_state == HybridState::TRANSITION_TO_ROVER) {
                float target_angle = _param_hybrid_ang_rov.get();
                if (fabsf(_current_mechanism_angle - target_angle) > tolerance) {
                    servos.control[0] = 1.0f;
                    servos.control[1] = -1.0f;
                } else {
                    servos.control[0] = -1.0f;
                    servos.control[1] = -1.0f;
                    _transformation_completed = true;
                }
            }
            else if (_current_state == HybridState::TRANSITION_TO_QUAD) {
                float target_angle = _param_hybrid_ang_qud.get();
                if (fabsf(_current_mechanism_angle - target_angle) > tolerance) {
                    servos.control[0] = -1.0f;
                    servos.control[1] = 1.0f;
                } else {
                    servos.control[0] = -1.0f;
                    servos.control[1] = -1.0f;
                    _transformation_completed = true;
                }
            }
        }
        else if (mag_qud_valid || mag_rov_valid) {
            // ---------------------------------------------------
            // 模式 B: TMAG5273 离散双限位开环/半闭环
            // ---------------------------------------------------

            if (_current_state == HybridState::TRANSITION_TO_ROVER) {
                // 确保目标方向的传感器没有掉线
                if (!mag_rov_valid) {
                    servos.control[0] = -1.0f; servos.control[1] = -1.0f;
                    _transformation_completed = false;
                }
                // 判定：使用 fabsf 获取绝对强度，防止 N/S 极装反导致读数为负
                else if (fabsf(_current_mag_z_rov) < _param_hyb_mag_thr_rov.get()) {
                    // 未到达限位，硬写死正向动力
                    servos.control[0] = 1.0f;
                    servos.control[1] = -1.0f;
                } else {
                    // 磁场强度超越阈值，刹车并确认到达
                    servos.control[0] = -1.0f;
                    servos.control[1] = -1.0f;
                    _transformation_completed = true;
                }
            }
            else if (_current_state == HybridState::TRANSITION_TO_QUAD) {
                if (!mag_qud_valid) {
                    servos.control[0] = -1.0f; servos.control[1] = -1.0f;
                    _transformation_completed = false;
                }
                else if (fabsf(_current_mag_z_qud) < _param_hyb_mag_thr_qud.get()) {
                    // 未到达限位，硬写死反向动力
                    servos.control[0] = -1.0f;
                    servos.control[1] = 1.0f;
                } else {
                    // 磁场强度超越阈值，刹车并确认到达
                    servos.control[0] = -1.0f;
                    servos.control[1] = -1.0f;
                    _transformation_completed = true;
                }
            }
        }
        else {
            // ---------------------------------------------------
            // 模式 C: 传感器全军覆没 (丢失保护)
            // ---------------------------------------------------
            servos.control[0] = -1.0f;
            servos.control[1] = -1.0f;

            static hrt_abstime last_warn_time = 0;
            if (hrt_elapsed_time(&last_warn_time) > 2_s) {
                PX4_ERR("[Hybrid] ALL SENSORS LOST! Auto-transform aborted. Use Manual Override!");
                last_warn_time = hrt_absolute_time();
            }
            _transformation_completed = false;
        }
    }

    // 只有在变形态才发布舵机指令（非变形态时电机由状态机保持锁死，防止空耗电）
    if (_manual_override_active ||
        _current_state == HybridState::TRANSITION_TO_ROVER ||
        _current_state == HybridState::TRANSITION_TO_QUAD) {
        _actuator_servos_pub.publish(servos);
    }
}

void HybridVehicleControl::updateParams()
{
	// 官方默认的参数更新
	ModuleParams::updateParams();

	// 绑定我们劫持的 QGC UI 参数
	if (_param_handle_rc_map_trans_sw == PARAM_INVALID) {
		_param_handle_rc_map_trans_sw = param_find("RC_MAP_TRANS_SW");
	}

	if (_param_handle_rc_map_trans_sw != PARAM_INVALID) {
		param_get(_param_handle_rc_map_trans_sw, &_rc_map_trans_sw_val);
	}
}
