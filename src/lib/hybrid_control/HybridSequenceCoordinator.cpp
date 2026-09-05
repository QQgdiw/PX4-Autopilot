#include "HybridSequenceCoordinator.hpp"

namespace hybrid_control
{

bool HybridSequenceCoordinator::stableFor(bool condition, uint64_t now_us, uint64_t duration_us, uint64_t &started_us)
{
	if (!condition) {
		started_us = 0;
		return false;
	}

	if (started_us == 0) {
		started_us = now_us;
	}

	return now_us - started_us >= duration_us;
}

void HybridSequenceCoordinator::enter(SequenceState state, uint64_t now_us)
{
	_output.state = state;
	_state_started_us = now_us;
	_landed_started_us = 0;
	_airborne_started_us = 0;
}

void HybridSequenceCoordinator::fail(SequenceFault fault, PropulsionOwner safe_owner)
{
	_output.state = SequenceState::Fault;
	_output.fault = fault;
	_output.propulsion_owner = safe_owner;
	_output.propulsion_ready = safe_owner != PropulsionOwner::None;
	_output.shape_request = HybridTarget::None;
	_output.gear_target = GearTarget::None;
	_output.request_disarm = false;
}

SequenceOutput HybridSequenceCoordinator::initialize(const SequenceConfig &config, const SequenceInput &input)
{
	_config = config;
	_output = {};
	_initialized = true;

	if (_config.gear_motion_timeout_us == 0 || _config.landed_debounce_us == 0
	    || _config.airborne_debounce_us == 0) {
		fail(SequenceFault::InvalidConfiguration, PropulsionOwner::None);
		return _output;
	}

	if (input.shape_state == HybridState::Flying) {
		enter(SequenceState::StableQuad, input.now_us);

	} else if (input.shape_state == HybridState::Driving) {
		enter(SequenceState::StableRover, input.now_us);

	} else {
		fail(SequenceFault::ShapeFault, PropulsionOwner::None);
	}

	refresh(input);
	return _output;
}

void HybridSequenceCoordinator::refresh(const SequenceInput &input)
{
	_output.shape_request = HybridTarget::None;
	_output.gear_target = GearTarget::None;
	_output.request_disarm = false;
	_output.propulsion_ready = false;

	switch (_output.state) {
	case SequenceState::StableQuad:
		_output.propulsion_owner = PropulsionOwner::Quad;
		_output.propulsion_ready = true;
		break;

	case SequenceState::QuadToRoverPrepare:
		_output.propulsion_owner = PropulsionOwner::Quad;
		_output.gear_target = _config.automatic_gear ? GearTarget::Down : GearTarget::None;
		break;

	case SequenceState::QuadToRoverTransform:
		_output.propulsion_owner = PropulsionOwner::None;
		_output.shape_request = HybridTarget::Driving;
		_output.gear_target = _config.automatic_gear ? GearTarget::Down : GearTarget::None;
		break;

	case SequenceState::RoverRetract:
		_output.propulsion_owner = input.gear_clear ? PropulsionOwner::Rover : PropulsionOwner::None;
		_output.propulsion_ready = input.gear_clear;
		_output.gear_target = _config.automatic_gear ? GearTarget::Stowed : GearTarget::None;
		break;

	case SequenceState::StableRover:
		_output.propulsion_owner = PropulsionOwner::Rover;
		_output.propulsion_ready = true;
		break;

	case SequenceState::RoverToQuadPrepare:
		_output.propulsion_owner = PropulsionOwner::None;
		_output.gear_target = _config.automatic_gear ? GearTarget::Down : GearTarget::None;
		_output.request_disarm = input.armed;
		break;

	case SequenceState::RoverToQuadTransform:
		_output.propulsion_owner = PropulsionOwner::None;
		_output.shape_request = HybridTarget::Flying;
		_output.gear_target = _config.automatic_gear ? GearTarget::Down : GearTarget::None;
		break;

	case SequenceState::QuadWaitAirborne:
		_output.propulsion_owner = PropulsionOwner::Quad;
		_output.propulsion_ready = true;
		_output.gear_target = _config.automatic_gear ? GearTarget::Down : GearTarget::None;
		break;

	case SequenceState::QuadRetract:
		_output.propulsion_owner = PropulsionOwner::Quad;
		_output.propulsion_ready = true;
		_output.gear_target = _config.automatic_gear ? GearTarget::Stowed : GearTarget::None;
		break;

	case SequenceState::Fault:
		break;
	}
}

SequenceOutput HybridSequenceCoordinator::update(const SequenceInput &input)
{
	if (!_initialized || _output.state == SequenceState::Fault) {
		return _output;
	}

	if (input.shape_fault != TransformFault::None) {
		fail(SequenceFault::ShapeFault, PropulsionOwner::None);
		return _output;
	}

	if (_output.state == SequenceState::StableQuad && input.requested_target == HybridTarget::Driving) {
		enter(SequenceState::QuadToRoverPrepare, input.now_us);

	} else if (_output.state == SequenceState::StableRover && input.requested_target == HybridTarget::Flying) {
		enter(SequenceState::RoverToQuadPrepare, input.now_us);
	}

	const bool gear_needed = _output.state == SequenceState::QuadToRoverPrepare
				 || _output.state == SequenceState::RoverToQuadPrepare
				 || _output.state == SequenceState::RoverRetract
				 || _output.state == SequenceState::QuadWaitAirborne
				 || _output.state == SequenceState::QuadRetract;

	if (gear_needed && (!input.gear_online || !input.gear_healthy)) {
		const PropulsionOwner safe_owner = _output.state == SequenceState::QuadToRoverPrepare
						   ? PropulsionOwner::Quad : PropulsionOwner::None;
		fail(SequenceFault::GearCommunication, safe_owner);
		return _output;
	}

	switch (_output.state) {
	case SequenceState::QuadToRoverPrepare: {
		const bool landed = stableFor(input.landed, input.now_us, _config.landed_debounce_us, _landed_started_us);

		if (_config.automatic_gear && !input.gear_down
		    && input.now_us - _state_started_us >= _config.gear_motion_timeout_us) {
			fail(SequenceFault::GearTimeout, PropulsionOwner::Quad);
			return _output;
		}

		if (landed && (!_config.automatic_gear || input.gear_down)) {
			if (input.armed) {
				refresh(input);
				_output.request_disarm = true;
				return _output;
			}

			enter(SequenceState::QuadToRoverTransform, input.now_us);
		}
		break;
	}

	case SequenceState::QuadToRoverTransform:
		if (input.shape_state == HybridState::Driving) {
			enter(_config.automatic_gear ? SequenceState::RoverRetract : SequenceState::StableRover,
			      input.now_us);
		}
		break;

	case SequenceState::RoverRetract:
		if (!input.gear_stowed && input.now_us - _state_started_us >= _config.gear_motion_timeout_us) {
			fail(SequenceFault::GearTimeout, input.gear_clear ? PropulsionOwner::Rover : PropulsionOwner::None);
			return _output;
		}

		if (input.gear_stowed) {
			enter(SequenceState::StableRover, input.now_us);
		}
		break;

	case SequenceState::RoverToQuadPrepare:
		if (_config.automatic_gear && !input.gear_down
		    && input.now_us - _state_started_us >= _config.gear_motion_timeout_us) {
			fail(SequenceFault::GearTimeout, PropulsionOwner::None);
			return _output;
		}

		if (!input.armed && (!_config.automatic_gear || input.gear_down)) {
			enter(SequenceState::RoverToQuadTransform, input.now_us);
		}
		break;

	case SequenceState::RoverToQuadTransform:
		if (input.shape_state == HybridState::Flying) {
			enter(_config.automatic_gear ? SequenceState::QuadWaitAirborne : SequenceState::StableQuad,
			      input.now_us);
		}
		break;

	case SequenceState::QuadWaitAirborne:
		if (stableFor(!input.landed, input.now_us, _config.airborne_debounce_us, _airborne_started_us)) {
			enter(SequenceState::QuadRetract, input.now_us);
		}
		break;

	case SequenceState::QuadRetract:
		if (!input.gear_stowed && input.now_us - _state_started_us >= _config.gear_motion_timeout_us) {
			fail(SequenceFault::GearTimeout, PropulsionOwner::Quad);
			return _output;
		}

		if (input.gear_stowed) {
			enter(SequenceState::StableQuad, input.now_us);
		}
		break;

	case SequenceState::StableQuad:
	case SequenceState::StableRover:
	case SequenceState::Fault:
		break;
	}

	refresh(input);
	return _output;
}

} // namespace hybrid_control
