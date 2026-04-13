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

	// 物理动作：在变形期间，直接驱动变形舵机 (这段保留)
	if (_current_state == HybridState::TRANSITION_TO_ROVER ||
		_current_state == HybridState::TRANSITION_TO_QUAD) {
		control_transformation_actuators();
	}
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
	// 来源 1：解析自动航线 (Mission) 中的 MAVLink 车辆指令
	// =========================================================
	vehicle_command_s vcmd{};
	while (_vehicle_command_sub.update(&vcmd)) {
		if (vcmd.command == vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION) {
			int transition_target = (int)(vcmd.param1 + 0.5f);
			if (is_in_auto_mode) {
				if (transition_target == 4) { request_rover = true; }
				else if (transition_target == 3) { request_quad = true; }
			}
		}
	}

	// =========================================================
	// 来源 2：解析遥控器拨杆 (跳变检测 + 主动降级)
	// =========================================================
	manual_control_setpoint_s manual_control;
	if (_manual_control_setpoint_sub.update(&manual_control)) {
		float current_main_switch = -1.0f;
		switch (_param_hybrid_rc_ch.get()) {
			case 1: current_main_switch = manual_control.aux1; break;
			case 2: current_main_switch = manual_control.aux2; break;
			case 3: current_main_switch = manual_control.aux3; break;
			case 4: current_main_switch = manual_control.aux4; break;
			case 5: current_main_switch = manual_control.aux5; break;
			case 6: current_main_switch = manual_control.aux6; break;
			default: current_main_switch = manual_control.aux1; break;
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

		static float last_main_switch = current_main_switch;
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
		if (fabsf(current_main_switch - last_main_switch) > 0.5f) {
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
			if (current_main_switch > 0.5f) {
				request_rover = true;
				request_quad = false;
			} else if (current_main_switch < -0.5f) {
				request_quad = true;
				request_rover = false;
			}
		}
		last_main_switch = current_main_switch;
		last_manual_switch = current_manual_switch;
		_manual_rc_value = current_manual_switch;	// 缓存供底盘执行器使用
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

	// =======================================================
	// 1. 获取并检查传感器状态 (超时机制)
	// =======================================================
	sensor_encoder_s encoder_data;
	if (_encoder_sub.update(&encoder_data)) {
		_current_mechanism_angle = encoder_data.position_rad;
		_last_encoder_timestamp = encoder_data.timestamp;
	}

	// 判断传感器是否失效 (超过 300ms 没有收到新数据即认为丢失)
	bool sensor_valid = (hrt_elapsed_time(&_last_encoder_timestamp) < 300000.0f);

	// =======================================================
	// 2. 动力输出仲裁逻辑
	// =======================================================

	if (_manual_override_active) {
		// 【最高优先级】物理拨杆直接接管
		// 将 _manual_rc_value (范围 -1 到 1) 互补输出
		servos.control[0] = _manual_rc_value;
		servos.control[1] = -_manual_rc_value;

		// 在手动接管期间，为了防止状态机死锁，强制不发送完成信号，
		// 载具会一直处于 TRANSITIONING 状态直到主开关被切回。
		_transformation_completed = false;
	}
	else {
		// 【自动闭环模式】
		if (!sensor_valid) {
			// [传感器失效/丢失保护]
			// 立即停止电机，防止机构撞毁
			servos.control[0] = -1.0f;
			servos.control[1] = -1.0f;

			// 限制报警频率，防止刷屏
			static hrt_abstime last_warn_time = 0;
			if (hrt_elapsed_time(&last_warn_time) > 2_s) {
				PX4_ERR("[Hybrid] ENCODER LOST! Auto-transform aborted. Use Manual Override!");
				last_warn_time = hrt_absolute_time();
			}
			_transformation_completed = false;
		}
		else {
			// [传感器正常：执行闭环位置控制 (Bang-Bang 控制示例)]
			// 容差设定 (例如 0.05 rad，约 3度)，防止在目标位置震荡
			const float tolerance = 0.05f;

			if (_current_state == HybridState::TRANSITION_TO_ROVER) {

				float target_angle = _param_hybrid_ang_rov.get();
				if (fabsf(_current_mechanism_angle - target_angle) > tolerance) {
					// 未到达位置，输出正向动力
					servos.control[0] = 1.0f;
					servos.control[1] = -1.0f;
				} else {
					// 到达目标！刹车并标记完成
					servos.control[0] = -1.0f;
					servos.control[1] = -1.0f;
					_transformation_completed = true;
				}

			}
			else if (_current_state == HybridState::TRANSITION_TO_QUAD) {

				float target_angle = _param_hybrid_ang_qud.get();
				if (fabsf(_current_mechanism_angle - target_angle) > tolerance) {
					// 未到达位置，输出反向动力
					servos.control[0] = -1.0f;
					servos.control[1] = 1.0f;
				} else {
					// 到达目标！刹车并标记完成
					servos.control[0] = -1.0f;
					servos.control[1] = -1.0f;
					_transformation_completed = true;
				}
			}
		}
	}

	// 只有在变形态才发布舵机指令（非变形态时电机由状态机保持锁死，防止空耗电）
	if (_current_state == HybridState::TRANSITION_TO_ROVER ||
		_current_state == HybridState::TRANSITION_TO_QUAD) {
		_actuator_servos_pub.publish(servos);
	}
}
