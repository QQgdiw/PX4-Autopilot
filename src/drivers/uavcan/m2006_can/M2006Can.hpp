#pragma once

#include <drivers/drv_hrt.h>
#include <lib/hybrid_control/C610Protocol.hpp>
#include <lib/hybrid_control/M2006CommandAdapter.hpp>
#include <lib/hybrid_control/M2006DriveGate.hpp>
#include <lib/hybrid_control/M2006SpeedController.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/hybrid_vehicle_status.h>
#include <uORB/topics/m2006_motor_status.h>
#include <uORB/topics/parameter_update.h>

#include "../uavcan_driver.hpp"

class M2006Can final : public ModuleBase<M2006Can>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	M2006Can();
	~M2006Can() override;

	static int task_spawn(int argc, char *argv[]);
	static M2006Can *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	void Run() override;
	int print_status() override;

private:
	static constexpr hrt_abstime RunIntervalUs{2000};
	static constexpr hrt_abstime StatusIntervalUs{20000};
	static constexpr unsigned TxFailureLimit{10};

	void updateControllerConfiguration();
	bool sendCommand(int16_t left, int16_t right);
	void sendZeroBestEffort();
	void receiveFeedback();
	void publishStatus(hrt_abstime now, const bool online[2]);

	UAVCAN_DRIVER::CanInitHelper<16> _can{1u};
	uavcan::ICanIface *_iface{nullptr};
	hybrid_control::M2006SpeedController _speed[2] {};
	hybrid_control::M2006DriveGate _gate{};

	uORB::Subscription _motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _hybrid_status_sub{ORB_ID(hybrid_vehicle_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1000000};
	uORB::Publication<m2006_motor_status_s> _status_pub{ORB_ID(m2006_motor_status)};

	actuator_motors_s _motors{};
	actuator_armed_s _armed{};
	hybrid_vehicle_status_s _hybrid_status{};
	hybrid_control::C610Feedback _feedback[2] {};
	uint64_t _feedback_timestamp[2] {};
	uint32_t _rx_count[2] {};
	bool _feedback_seen[2] {};
	bool _online_previous[2] {};
	int16_t _current_command[2] {};
	uint8_t _left_id{0};
	uint8_t _right_id{0};

	hrt_abstime _last_run{0};
	hrt_abstime _last_status_publish{0};
	uint32_t _tx_count{0};
	uint32_t _tx_full_count{0};
	uint32_t _tx_error_count{0};
	uint32_t _timeout_count{0};
	unsigned _consecutive_tx_failures{0};
	uint64_t _last_can_error_count{0};
	bool _rx_error{false};
	bool _controller_config_valid{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::M2K_EN>) _param_enable,
		(ParamInt<px4::params::M2K_L_ID>) _param_left_id,
		(ParamInt<px4::params::M2K_R_ID>) _param_right_id,
		(ParamInt<px4::params::M2K_L_REV>) _param_left_reverse,
		(ParamInt<px4::params::M2K_R_REV>) _param_right_reverse,
		(ParamFloat<px4::params::M2K_MAX_RPM>) _param_max_rpm,
		(ParamInt<px4::params::M2K_CUR_LIM>) _param_current_limit,
		(ParamFloat<px4::params::M2K_SPD_P>) _param_speed_p,
		(ParamFloat<px4::params::M2K_SPD_I>) _param_speed_i,
		(ParamFloat<px4::params::M2K_SPD_D>) _param_speed_d,
		(ParamFloat<px4::params::M2K_SPD_FF>) _param_speed_ff,
		(ParamFloat<px4::params::M2K_RPM_SLEW>) _param_rpm_slew,
		(ParamFloat<px4::params::M2K_FB_TO>) _param_feedback_timeout,
		(ParamFloat<px4::params::M2K_CMD_TO>) _param_command_timeout
	)
};
