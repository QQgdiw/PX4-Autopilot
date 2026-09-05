#pragma once

#include "TransformationStateMachine.hpp"

#include <cstdint>

namespace hybrid_control
{

enum class SequenceState : uint8_t {
	StableQuad,
	QuadToRoverPrepare,
	QuadToRoverTransform,
	RoverRetract,
	StableRover,
	RoverToQuadPrepare,
	RoverToQuadTransform,
	QuadWaitAirborne,
	QuadRetract,
	Fault
};

enum class PropulsionOwner : uint8_t { None, Quad, Rover };
enum class GearTarget : uint8_t { None, Down, Stowed };
enum class SequenceFault : uint8_t { None, GearCommunication, GearTimeout, ShapeFault, InvalidConfiguration };

struct SequenceConfig {
	bool automatic_gear{true};
	uint64_t gear_motion_timeout_us{8000000};
	uint64_t landed_debounce_us{1000000};
	uint64_t airborne_debounce_us{1000000};
};

struct SequenceInput {
	uint64_t now_us{0};
	HybridTarget requested_target{HybridTarget::None};
	HybridState shape_state{HybridState::Unknown};
	TransformFault shape_fault{TransformFault::None};
	bool armed{false};
	bool landed{false};
	bool gear_online{false};
	bool gear_healthy{false};
	bool gear_down{false};
	bool gear_clear{false};
	bool gear_stowed{false};
};

struct SequenceOutput {
	SequenceState state{SequenceState::Fault};
	HybridTarget shape_request{HybridTarget::None};
	PropulsionOwner propulsion_owner{PropulsionOwner::None};
	GearTarget gear_target{GearTarget::None};
	SequenceFault fault{SequenceFault::None};
	bool propulsion_ready{false};
	bool request_disarm{false};
};

class HybridSequenceCoordinator
{
public:
	SequenceOutput initialize(const SequenceConfig &config, const SequenceInput &input);
	SequenceOutput update(const SequenceInput &input);
	const SequenceOutput &output() const { return _output; }

private:
	bool stableFor(bool condition, uint64_t now_us, uint64_t duration_us, uint64_t &started_us);
	void enter(SequenceState state, uint64_t now_us);
	void fail(SequenceFault fault, PropulsionOwner safe_owner);
	void refresh(const SequenceInput &input);

	SequenceConfig _config{};
	SequenceOutput _output{};
	uint64_t _state_started_us{0};
	uint64_t _landed_started_us{0};
	uint64_t _airborne_started_us{0};
	bool _initialized{false};
};

} // namespace hybrid_control
