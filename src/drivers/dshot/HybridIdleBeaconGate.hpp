#pragma once

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/hybrid_vehicle_status.h>
#include <uORB/topics/vehicle_status.h>

// Driver-side adapter: the core timer has no uORB or hardware dependencies.
inline bool hybridIdleBeaconEligible(uint64_t now, const vehicle_status_s &vehicle,
				     const hybrid_vehicle_status_s &hybrid, const actuator_armed_s &armed,
				     bool stop_motors, bool rotors_stopped, uint8_t rotor_functions, bool command_busy)
{
	const auto fresh = [now](uint64_t timestamp) {
		return timestamp != 0 && timestamp <= now && now - timestamp <= 500000;
	};
	return fresh(vehicle.timestamp) && vehicle.is_quad_rover
	       && fresh(hybrid.timestamp) && hybrid.current_state == hybrid_vehicle_status_s::HYBRID_STATE_FLYING
	       && hybrid.fault_reason == 0 && hybrid.sequence_fault == 0 && hybrid.propulsion_ready
	       && hybrid.position_confirmed && hybrid.actuator_online && hybrid.actuator_healthy
	       && hybrid.actuator_config_verified && hybrid.actuator_protection_flags == 0
	       && (hybrid.actuator_backend != hybrid_vehicle_status_s::ACTUATOR_HX65
		   || ((hybrid.sequence_state == hybrid_vehicle_status_s::SEQUENCE_STABLE_QUAD
			|| (hybrid.sequence_state == hybrid_vehicle_status_s::SEQUENCE_QUAD_WAIT_AIRBORNE && hybrid.gear_down))
		       && hybrid.propulsion_owner == hybrid_vehicle_status_s::PROPULSION_QUAD
		       && hybrid.gear_online && hybrid.gear_healthy))
	       && fresh(armed.timestamp) && !armed.armed && !armed.prearmed
	       && !armed.lockdown && !armed.manual_lockdown && !armed.force_failsafe
	       && !armed.in_esc_calibration_mode && stop_motors && rotors_stopped && rotor_functions == 0x0f
	       && !command_busy;
}
