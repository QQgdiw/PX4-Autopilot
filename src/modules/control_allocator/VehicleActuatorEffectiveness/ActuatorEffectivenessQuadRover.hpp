#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessRoverDifferential.hpp"
#include "ActuatorEffectivenessControlSurfaces.hpp"

class ActuatorEffectivenessQuadRover : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessQuadRover(ModuleParams *parent);
	virtual ~ActuatorEffectivenessQuadRover() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
			    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) override;

	const char *name() const override { return "QuadRover"; }

protected:
	// 实例化两个物理子模块
	ActuatorEffectivenessRotors _mc_motors;
	ActuatorEffectivenessRoverDifferential _rover_motors;

	// 仅记录多旋翼电机的掩码（用于降落停转保护）
	uint32_t _mc_motors_mask{};
};
