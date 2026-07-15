/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "hybridCheck.hpp"

#include <cmath>

#include <parameters/param.h>
#include <px4_platform_common/events.h>

using namespace time_literals;

namespace
{

constexpr hrt_abstime StatusTimeout{500_ms};
constexpr int32_t Servo1Function{201};

bool getIntParameter(const char *name, int32_t &value)
{
	const param_t param = param_find(name);
	return param != PARAM_INVALID && param_get(param, &value) == PX4_OK;
}

bool getFloatParameter(const char *name, float &value)
{
	const param_t param = param_find(name);
	return param != PARAM_INVALID && param_get(param, &value) == PX4_OK;
}

bool isFresh(uint64_t timestamp)
{
	return timestamp != 0 && hrt_absolute_time() - timestamp <= StatusTimeout;
}

} // namespace

void HybridChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (!context.status().is_quad_rover) {
		return;
	}

	hybrid_vehicle_status_s hybrid_status{};
	HybridCheckConfiguration configuration{};
	const bool have_configuration = getConfiguration(configuration);
	const bool have_hybrid_status = _hybrid_status_sub.copy(&hybrid_status);
	const bool stable_flying = have_hybrid_status && isFresh(hybrid_status.timestamp)
				   && hybrid_status.current_state == hybrid_vehicle_status_s::HYBRID_STATE_FLYING
				   && hybrid_status.fault_reason == hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE;
	const bool stable_driving = have_hybrid_status && isFresh(hybrid_status.timestamp)
				    && hybrid_status.current_state == hybrid_vehicle_status_s::HYBRID_STATE_DRIVING
				    && hybrid_status.fault_reason == hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE;

	if (!stable_flying && !stable_driving) {
		/* EVENT
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::system,
					    events::ID("check_hybrid_state_unsafe"), events::Log::Error,
					    "Hybrid mechanism state is unsafe for arming");
		reporter.clearCanRunBits(NavModes::All);
	}

	if (!have_configuration || !hasSafeServoMapping(configuration)) {
		/* EVENT
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::system,
					    events::ID("check_hybrid_servo_mapping"), events::Log::Error,
					    "Hybrid M8 servo safety configuration is invalid");
		reporter.clearCanRunBits(NavModes::All);
	}

	if (stable_driving) {
		m2006_motor_status_s m2006_status{};
		const bool have_m2006_status = _m2006_status_sub.copy(&m2006_status);

		if (!have_configuration || configuration.m2006_enabled == 0 || !have_m2006_status
		    || !isM2006Healthy(m2006_status) || hasM2006Conflict(configuration)) {
			/* EVENT
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::system,
						    events::ID("check_hybrid_m2006_unhealthy"), events::Log::Error,
						    "Hybrid M2006 CAN wheel drive is not healthy");
			reporter.clearCanRunBits(NavModes::All);
		}

		if (!have_configuration || !hasConfiguredSpeedController(configuration)) {
			/* EVENT
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::system,
						    events::ID("check_hybrid_controller_unconfigured"), events::Log::Error,
						    "Hybrid M2006 speed controller is unconfigured");
			reporter.clearCanRunBits(NavModes::All);
		}
	}
}

bool HybridChecks::getConfiguration(HybridCheckConfiguration &configuration) const
{
	if (_test_configuration.servo_function != 0 || _test_configuration.m2006_enabled != 0) {
		configuration = _test_configuration;
		return true;
	}

	return getIntParameter("M2K_EN", configuration.m2006_enabled)
	       && getIntParameter("UAVCAN_ENABLE", configuration.uavcan_enabled)
	       && getIntParameter("CYPHAL_ENABLE", configuration.cyphal_enabled)
	       && getIntParameter("PWM_MAIN_FUNC5", configuration.motor5_function)
	       && getIntParameter("PWM_MAIN_FUNC6", configuration.motor6_function)
	       && getIntParameter("PWM_MAIN_FUNC8", configuration.servo_function)
	       && getIntParameter("PWM_MAIN_DIS8", configuration.m8_disarmed)
	       && getIntParameter("PWM_MAIN_FAIL8", configuration.m8_failsafe)
	       && getFloatParameter("HYB_SV_QUD", configuration.quad_servo_target)
	       && getFloatParameter("HYB_SV_ROV", configuration.rover_servo_target)
	       && getFloatParameter("M2K_SPD_P", configuration.speed_p)
	       && getFloatParameter("M2K_SPD_I", configuration.speed_i)
	       && getFloatParameter("M2K_SPD_FF", configuration.speed_ff);
}

bool HybridChecks::hasSafeServoMapping(const HybridCheckConfiguration &configuration) const
{
	return configuration.motor5_function == 0 && configuration.motor6_function == 0
	       && configuration.servo_function == Servo1Function && configuration.m8_disarmed == 0
	       && configuration.m8_failsafe == 0 && std::isfinite(configuration.quad_servo_target)
	       && std::isfinite(configuration.rover_servo_target)
	       && configuration.quad_servo_target >= -1.f && configuration.quad_servo_target <= 1.f
	       && configuration.rover_servo_target >= -1.f && configuration.rover_servo_target <= 1.f
	       && std::fabs(configuration.quad_servo_target - configuration.rover_servo_target) > 0.001f;
}

bool HybridChecks::hasConfiguredSpeedController(const HybridCheckConfiguration &configuration) const
{
	return std::isfinite(configuration.speed_p) && std::isfinite(configuration.speed_i)
	       && std::isfinite(configuration.speed_ff)
	       && (std::fabs(configuration.speed_p) > 0.0001f || std::fabs(configuration.speed_i) > 0.0001f
		   || std::fabs(configuration.speed_ff) > 0.0001f);
}

bool HybridChecks::hasM2006Conflict(const HybridCheckConfiguration &configuration) const
{
	return configuration.uavcan_enabled != 0 || configuration.cyphal_enabled != 0;
}

bool HybridChecks::isM2006Healthy(const m2006_motor_status_s &status) const
{
	return isFresh(status.timestamp) && status.online[0] && status.online[1]
	       && status.fault_flags == m2006_motor_status_s::DRIVE_FAULT_NONE;
}
