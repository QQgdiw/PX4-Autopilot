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
#include <lib/hybrid_control/TransformationStateMachine.hpp>
#include <lib/hybrid_control/M2006DriveGate.hpp>
#include <lib/hybrid_control/Hx8BackendPolicy.hpp>
#include <lib/hybrid_control/Hx65BackendPolicy.hpp>
#include <lib/hybrid_control/HybridSequenceCoordinator.hpp>
#include <uORB/SubscriptionMultiArray.hpp>

// uORB 发布与订阅
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/manual_control_switches.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/hybrid_vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/sensor_encoder.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/magnetic_sensor.h>
#include <uORB/topics/hx8_servo_status.h>
#include <uORB/topics/hx8_servo_command.h>
#include <uORB/topics/hx65_servo_status.h>
#include <uORB/topics/hx65_servo_command.h>

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
	void update_state_machine(const hybrid_control::TransformationInput &input);
	hybrid_control::TransformationInput update_transformation_input(
		hrt_abstime now, const hybrid_control::TransformationConfig &config);
	void publish_status(const hybrid_control::TransformationInput &input, hrt_abstime now);
	void publish_servo(hrt_abstime now);
	void publish_hx8_command(hrt_abstime now);
	void publish_hx65_command(hrt_abstime now);
	void publish_gear_command(hrt_abstime now);
	void publish_motor_outputs(hrt_abstime now);
	hybrid_control::TransformationConfig transformation_config() const;
	int clear_fault();
	bool selected_feedback_fresh(hrt_abstime now, const hybrid_control::TransformationConfig &config) const;
	bool transformation_pwm_command_effective() const;

	/**
	 * 检查变形条件是否安全 (例如: 必须贴地才能变形为车)
	 * @return true 如果允许变形
	 */
	bool check_safe_to_transform(bool to_rover);

	// === uORB 订阅 (获取系统状态与遥控器输入) ===
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _manual_control_switches_sub{ORB_ID(manual_control_switches)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _encoder_sub{ORB_ID(sensor_encoder)};
	uORB::SubscriptionMultiArray<magnetic_sensor_s, 2> _magnetic_subs{ORB_ID::magnetic_sensor};
	uORB::Subscription _hx8_status_sub{ORB_ID(hx8_servo_status)};
	uORB::Subscription _hx65_status_sub{ORB_ID(hx65_servo_status)};
	// 监听多旋翼分配器发出的电机指令 (Motor 1-4)
	uORB::Subscription _actuator_motors_mc_sub{ORB_ID(actuator_motors_mc)};
	// 监听官方差速模块发出的车轮指令 (Motor 5-6)
	uORB::Subscription _actuator_motors_rover_sub{ORB_ID(actuator_motors_rover)};

	// === uORB 发布 (输出控制指令) ===
	uORB::Publication<actuator_servos_s>      _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<hybrid_vehicle_status_s> _hybrid_status_pub{ORB_ID(hybrid_vehicle_status)};
	uORB::Publication<vehicle_command_s>      _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<hx8_servo_command_s> _hx8_command_pub{ORB_ID(hx8_servo_command)};
	uORB::Publication<hx65_servo_command_s> _hx65_command_pub{ORB_ID(hx65_servo_command)};
	// 全系统唯一允许向最终物理引脚发送电机指令的模块
	uORB::Publication<actuator_motors_s> _actuator_motors_final_pub{ORB_ID(actuator_motors)};

	// 监听 MAVLink 车辆指令 (用于响应航线中的切换节点和地面站滑块)
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	// 用于向地面站回复 "指令已收到并执行"，避免 QGC 报错超时
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};

	uint8_t _last_transition_switch{manual_control_switches_s::SWITCH_POS_NONE};

	// 缓存结构体
	actuator_motors_s _mc_motors{};
	actuator_motors_s _rover_motors{};

	// === 内部状态变量 ===
	hybrid_control::TransformationStateMachine _transformation;
	hybrid_control::TransformationConfigTracker _transformation_config_tracker;
	hybrid_control::TransformationOutput _transformation_output{hybrid_control::HybridState::Unknown,
			       hybrid_control::HybridTarget::None, hybrid_control::SensorSource::None,
			       hybrid_control::TransformFault::None, false, false, 0.f};
	bool _transformation_initialized{false};
	hrt_abstime _transition_start_time{0};
	bool _transition_timing_active{false};
	// Startup-only diagnostic probe. It records event edges only and never changes
	// transition, actuator, or fault behavior.
	hrt_abstime _startup_probe_started{0};
	bool _startup_probe_hx8_seen{false};
	bool _startup_probe_hx8_online{false};
	bool _startup_probe_hx8_healthy{false};
	bool _startup_probe_hx8_verified{false};
	bool _startup_probe_hx8_command_accepted{false};
	uint8_t _startup_probe_hx8_result{0};
	uint32_t _startup_probe_hx8_sequence{0};
	bool _startup_probe_first_rc_logged{false};
	bool _startup_probe_fault_logged{false};
	actuator_armed_s _actuator_armed{};
	vehicle_land_detected_s _vehicle_land_detected{};
	hx8_servo_status_s _hx8_status{};
	hx65_servo_status_s _hx65_status{};
	uint32_t _hx8_sequence{0};
	hybrid_control::HybridTarget _hx8_last_target{hybrid_control::HybridTarget::None};
	uint64_t _hx8_last_hold{0};
	uint8_t _hx8_release_attempts{0};
	hybrid_control::Hx8CommandPolicy _hx8_command_policy;
	hybrid_control::Hx8CommandPolicy _hx65_command_policy;
	hybrid_control::HybridSequenceCoordinator _sequence;
	hybrid_control::SequenceOutput _sequence_output{};
	bool _sequence_initialized{false};
	uint32_t _gear_sequence{0};
	hybrid_control::GearTarget _last_gear_target{hybrid_control::GearTarget::None};
	uint64_t _last_disarm_request{0};
	float _manual_gear_value{0.f};
	uint64_t _manual_gear_timestamp{0};

	bool _manual_commissioning_active{false};
	bool _manual_value_initialized{false};
	float _last_manual_value{0.f};
	hybrid_control::ManualControlCache _manual_control_cache;

	float _current_mechanism_angle{0.0f};   // 当前变形机构的角度 (rad)
	uint64_t _last_encoder_timestamp{0};    // 上一次收到编码器数据的时间戳
	bool _encoder_healthy{false};
	hybrid_control::TmagSampleCache _tmag_quad_cache;
	hybrid_control::TmagSampleCache _tmag_rover_cache;
	hybrid_control::TmagRatioFilter _tmag_ratio_filter;
	hybrid_control::SensorSource _position_source{hybrid_control::SensorSource::None};

	vehicle_control_mode_s _vcontrol_mode{};         // 缓存控制模式数据

	// === 参数绑定 (通过 ModuleParams) ===
	// 注意：这些参数将在 .c 文件中进行系统级注册
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::HYBRID_TRANS_T>) 	_param_hybrid_trans_t, // 变形动作所需的物理时间(秒)
		(ParamBool<px4::params::HYB_SENS_EN>)		_param_hyb_sens_en,
		(ParamInt<px4::params::HYB_BOOT_ST>)		_param_hyb_boot_st,
		(ParamFloat<px4::params::HYB_SV_QUD>)		_param_hyb_sv_qud,
		(ParamFloat<px4::params::HYB_SV_ROV>)		_param_hyb_sv_rov,
		(ParamFloat<px4::params::HYB_ANG_TOL>)		_param_hyb_ang_tol,
		(ParamFloat<px4::params::HYB_SENS_TO>)		_param_hyb_sens_to,
		(ParamFloat<px4::params::HYB_DBNC_T>)		_param_hyb_dbnc_t,
		(ParamInt<px4::params::HYBRID_MAN_CH>)    	_param_hybrid_man_ch,  // 手动接管机构的 AUX 通道 (1-6)
		(ParamFloat<px4::params::HYBRID_ANG_ROV>) 	_param_hybrid_ang_rov, // 车模式的机构目标角度 (rad)
		(ParamFloat<px4::params::HYBRID_ANG_QUD>)	_param_hybrid_ang_qud,  // 飞机模式的机构目标角度 (rad)
		(ParamInt<px4::params::HYB_MAG_ID_QUD>) 	_param_hyb_mag_id_qud,
		(ParamInt<px4::params::HYB_MAG_ID_ROV>) 	_param_hyb_mag_id_rov,
		(ParamFloat<px4::params::HYB_MAG_THR_QUD>) 	_param_hyb_mag_thr_qud,
		(ParamFloat<px4::params::HYB_MAG_THR_ROV>) 	_param_hyb_mag_thr_rov
		,(ParamInt<px4::params::HYB_ACT_TYPE>) _param_hyb_act_type
		,(ParamFloat<px4::params::HYB_STALL_T>) _param_hyb_stall_t
		,(ParamFloat<px4::params::HYB_STALL_D>) _param_hyb_stall_d
		,(ParamInt<px4::params::HX8_ID>) _param_hx8_id
		,(ParamFloat<px4::params::HX8_ANG_QUD>) _param_hx8_ang_qud
		,(ParamFloat<px4::params::HX8_ANG_ROV>) _param_hx8_ang_rov
		,(ParamInt<px4::params::HX8_MOVE_T>) _param_hx8_move_t
		,(ParamInt<px4::params::HX8_ACC_T>) _param_hx8_acc_t
		,(ParamInt<px4::params::HX8_DEC_T>) _param_hx8_dec_t
		,(ParamInt<px4::params::HX8_PWR_LIM>) _param_hx8_pwr_lim
		,(ParamBool<px4::params::LG_AUTO_EN>) _param_lg_auto_en
		,(ParamInt<px4::params::LG_MAN_CH>) _param_lg_man_ch
		,(ParamFloat<px4::params::LG_ANG_DN>) _param_lg_ang_dn
		,(ParamFloat<px4::params::LG_ANG_CLR>) _param_lg_ang_clr
		,(ParamFloat<px4::params::LG_ANG_STW>) _param_lg_ang_stw
		,(ParamFloat<px4::params::LG_ANG_TOL>) _param_lg_ang_tol
		,(ParamFloat<px4::params::LG_TIMEOUT>) _param_lg_timeout
		,(ParamFloat<px4::params::LG_LAND_T>) _param_lg_land_t
		,(ParamFloat<px4::params::LG_AIR_T>) _param_lg_air_t
		,(ParamInt<px4::params::LG_MOVE_T>) _param_lg_move_t
		,(ParamInt<px4::params::LG_ACC_T>) _param_lg_acc_t
		,(ParamInt<px4::params::LG_DEC_T>) _param_lg_dec_t
		,(ParamInt<px4::params::LG_PWR_LIM>) _param_lg_pwr_lim
		,(ParamInt<px4::params::H65_L_ID>) _param_h65_l_id
		,(ParamInt<px4::params::H65_R_ID>) _param_h65_r_id
		,(ParamInt<px4::params::H65_L_QUD>) _param_h65_l_qud
		,(ParamInt<px4::params::H65_L_ROV>) _param_h65_l_rov
		,(ParamInt<px4::params::H65_R_QUD>) _param_h65_r_qud
		,(ParamInt<px4::params::H65_R_ROV>) _param_h65_r_rov
		,(ParamInt<px4::params::H65_SPEED>) _param_h65_speed
		,(ParamInt<px4::params::H65_ACC>) _param_h65_acc
		,(ParamInt<px4::params::H65_TOL>) _param_h65_tol
		,(ParamFloat<px4::params::H65_SKEW>) _param_h65_skew
	)
};

// 导出主函数入口，供系统底层调用
extern "C" __EXPORT int hybrid_vehicle_control_main(int argc, char *argv[]);
