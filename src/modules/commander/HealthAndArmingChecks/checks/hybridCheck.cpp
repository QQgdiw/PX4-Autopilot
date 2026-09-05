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

bool getOptionalIntParameter(const char *name, int32_t &value)
{
	const param_t param = param_find(name);

	if (param == PARAM_INVALID) {
		value = 0;
		return true;
	}

	return param_get(param, &value) == PX4_OK;
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

bool isStableAndSafe(const hybrid_vehicle_status_s &status)
{
	const bool stable_state = status.current_state == hybrid_vehicle_status_s::HYBRID_STATE_FLYING
				  || status.current_state == hybrid_vehicle_status_s::HYBRID_STATE_DRIVING;

	if (!stable_state || status.fault_reason != hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE
	    || status.sequence_fault != hybrid_vehicle_status_s::SEQUENCE_FAULT_NONE) {
		return false;
	}

	if (status.actuator_backend == hybrid_vehicle_status_s::ACTUATOR_PWM) {
		return !status.sensors_enabled || status.position_confirmed;
	}

	if (status.actuator_backend == hybrid_vehicle_status_s::ACTUATOR_HX8) {
		return status.position_confirmed && status.position_valid && std::isfinite(status.position_normalized)
		       && status.position_normalized >= 0.f && status.position_normalized <= 1.f
		       && status.actuator_online && status.actuator_healthy && status.actuator_config_verified
		       && status.actuator_protection_flags == 0;
	}

	if (status.actuator_backend == hybrid_vehicle_status_s::ACTUATOR_HX65) {
		return status.propulsion_ready && status.position_confirmed && status.position_valid
		       && std::isfinite(status.position_normalized)
		       && status.actuator_online && status.actuator_healthy && status.actuator_config_verified
		       && status.actuator_protection_flags == 0 && status.gear_online && status.gear_healthy;
	}

	return false;
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
	const bool stable_and_safe = have_hybrid_status && isFresh(hybrid_status.timestamp) && isStableAndSafe(hybrid_status);
	const bool stable_flying = stable_and_safe
				   && hybrid_status.current_state == hybrid_vehicle_status_s::HYBRID_STATE_FLYING;
	const bool stable_driving = stable_and_safe
				    && hybrid_status.current_state == hybrid_vehicle_status_s::HYBRID_STATE_DRIVING;

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
	       && getOptionalIntParameter("HYB_ACT_TYPE", configuration.actuator_backend)
	       && getIntParameter("UAVCAN_ENABLE", configuration.uavcan_enabled)
	       && getOptionalIntParameter("CYPHAL_ENABLE", configuration.cyphal_enabled)
	       && getIntParameter("PWM_MAIN_FUNC5", configuration.motor5_function)
	       && getIntParameter("PWM_MAIN_FUNC6", configuration.motor6_function)
	       && getIntParameter("PWM_MAIN_FUNC8", configuration.servo_function)
	       && getIntParameter("PWM_MAIN_DIS8", configuration.m8_disarmed)
	       && getIntParameter("PWM_MAIN_FAIL8", configuration.m8_failsafe)
	       && getFloatParameter("HYB_SV_QUD", configuration.quad_servo_target)
	       && getFloatParameter("HYB_SV_ROV", configuration.rover_servo_target)
	       && getFloatParameter("M2K_SPD_P", configuration.speed_p)
	       && getFloatParameter("M2K_SPD_I", configuration.speed_i)
	       && getFloatParameter("M2K_SPD_D", configuration.speed_d)
	       && getFloatParameter("M2K_SPD_FF", configuration.speed_ff);
}

bool HybridChecks::hasSafeServoMapping(const HybridCheckConfiguration &configuration) const
{
	if (configuration.actuator_backend == hybrid_vehicle_status_s::ACTUATOR_HX8
	    || configuration.actuator_backend == hybrid_vehicle_status_s::ACTUATOR_HX65) {
		return configuration.motor5_function == 0 && configuration.motor6_function == 0
		       && configuration.servo_function == 0 && configuration.m8_disarmed == 0
		       && configuration.m8_failsafe == 0;
	}

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
	return std::isfinite(configuration.speed_p) && configuration.speed_p >= 0.f
	       && std::isfinite(configuration.speed_i) && configuration.speed_i >= 0.f
	       && std::isfinite(configuration.speed_d) && configuration.speed_d >= 0.f
	       && std::isfinite(configuration.speed_ff) && configuration.speed_ff >= 0.f
	       && (configuration.speed_p > 0.0001f || configuration.speed_i > 0.0001f
		   || configuration.speed_ff > 0.0001f);
}

bool HybridChecks::hasM2006Conflict(const HybridCheckConfiguration &configuration) const
{
	return configuration.cyphal_enabled != 0;
}

bool HybridChecks::isM2006Healthy(const m2006_motor_status_s &status) const
{
	return isFresh(status.timestamp) && status.online[0] && status.online[1]
	       && status.fault_flags == m2006_motor_status_s::DRIVE_FAULT_NONE;
}
