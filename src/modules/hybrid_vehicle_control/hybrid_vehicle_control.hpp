/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 * used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file HybridVehicleControl.hpp
 *
 * Quad-Rover 混合载具变形仲裁与状态机控制模块。
 * 负责在四旋翼和漫游车模式之间平滑切换，并管理控制权的接管。
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// uORB 发布与订阅
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/hybrid_vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/sensor_encoder.h>
#include <uORB/topics/actuator_motors.h>

// 定义混合载具的状态机枚举
enum class HybridState {
	FLYING = 0,             // 纯多旋翼飞行模式
	TRANSITION_TO_ROVER,    // 正在向漫游车变形
	DRIVING,                // 纯地面漫游车模式
	TRANSITION_TO_QUAD      // 正在向多旋翼变形
};

class HybridVehicleControl : public ModuleBase<HybridVehicleControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	HybridVehicleControl();
	~HybridVehicleControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	/**
	 * 主循环 (运行在 Work Queue 线程中)
	 */
	void Run() override;

	/**
	 * 核心状态机更新逻辑
	 */
	void update_state_machine();

	/**
	 * 检查变形条件是否安全 (例如: 必须贴地才能变形为车)
	 * @return true 如果允许变形
	 */
	bool check_safe_to_transform(bool to_rover);

	/**
	 * 接管并发布特定的控制模式，启用/禁用底层模块
	 */
	void publish_control_mode();

	/**
	 * 在变形期间直接控制物理舵机
	 */
	void control_transformation_actuators();

	// === uORB 订阅 (获取系统状态与遥控器输入) ===
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _encoder_sub{ORB_ID(sensor_encoder)};
	// 监听多旋翼分配器发出的电机指令 (Motor 1-4)
	uORB::Subscription _actuator_motors_mc_sub{ORB_ID(actuator_motors_mc)};
	// 监听官方差速模块发出的车轮指令 (Motor 5-6)
	uORB::Subscription _actuator_motors_rover_sub{ORB_ID(actuator_motors_rover)};

	// === uORB 发布 (输出控制指令) ===
	uORB::Publication<actuator_servos_s>      _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<hybrid_vehicle_status_s> _hybrid_status_pub{ORB_ID(hybrid_vehicle_status)};
	uORB::Publication<vehicle_command_s>      _vehicle_command_pub{ORB_ID(vehicle_command)};
	// 全系统唯一允许向最终物理引脚发送电机指令的模块
	uORB::Publication<actuator_motors_s> _actuator_motors_final_pub{ORB_ID(actuator_motors)};

	// 缓存结构体
	actuator_motors_s _mc_motors{};
	actuator_motors_s _rover_motors{};

	// === 内部状态变量 ===
	HybridState _current_state{HybridState::FLYING}; // 默认开机假定为飞行形态
	hrt_abstime _transition_start_time{0};           // 记录变形开始的时间戳
	bool        _is_armed{false};                    // 当前是否处于解锁状态

	bool  _manual_override_active{false};   // 标记当前是否处于手动接管变形状态
	bool  _transformation_completed{false}; // 标记传感器是否确认变形到位

	float _current_mechanism_angle{0.0f};   // 当前变形机构的角度 (rad)
	uint64_t _last_encoder_timestamp{0};    // 上一次收到编码器数据的时间戳
	float _manual_rc_value{0.0f};           // 缓存手动接管通道的值

	vehicle_control_mode_s _vcontrol_mode{};         // 缓存控制模式数据

	// === 参数绑定 (通过 ModuleParams) ===
	// 注意：这些参数将在 .c 文件中进行系统级注册
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::HYBRID_TRANS_T>) 	_param_hybrid_trans_t, // 变形动作所需的物理时间(秒)
		(ParamFloat<px4::params::HYBRID_MAX_Z>)   	_param_hybrid_max_z,   // 允许变形为车模式的最大高度(米)
		(ParamInt<px4::params::HYBRID_RC_CH>)     	_param_hybrid_rc_ch,    // 遥控器上触发变形的辅助通道号 (1-6)
		(ParamInt<px4::params::HYBRID_MAN_CH>)    	_param_hybrid_man_ch,  // 手动接管机构的 AUX 通道 (如 5)
		(ParamFloat<px4::params::HYBRID_ANG_ROV>) 	_param_hybrid_ang_rov, // 车模式的机构目标角度 (rad)
		(ParamFloat<px4::params::HYBRID_ANG_QUD>)	_param_hybrid_ang_qud  // 飞机模式的机构目标角度 (rad)
	)
};

// 导出主函数入口，供系统底层调用
extern "C" __EXPORT int hybrid_vehicle_control_main(int argc, char *argv[]);
