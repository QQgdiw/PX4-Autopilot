#include "ActuatorEffectivenessQuadRover.hpp"

using namespace matrix;

ActuatorEffectivenessQuadRover::ActuatorEffectivenessQuadRover(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_motors(this),
	  _rover_motors(this)
{
}

bool
ActuatorEffectivenessQuadRover::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// =======================================================
	// 1. 添加多旋翼悬停电机 (Motor 1 ~ 4)
	// =======================================================
	// 启用螺旋桨反扭矩计算 (Quadrotor 必须依赖反扭矩来实现 Yaw 控制)
	_mc_motors.enablePropellerTorque(true);
	const bool mc_added_successfully = _mc_motors.addActuators(configuration);

	// 获取多旋翼电机的掩码，稍后用于 updateSetpoint
	_mc_motors_mask = _mc_motors.getMotors();

	// =======================================================
	// 2. 添加漫游车差速驱动轮 (Motor 5 ~ 6)
	// =======================================================
	// 这会自动读取 CA_R_REV 等车轮专属参数，并将其映射到 Thrust X 和 Yaw
	const bool rover_added_successfully = _rover_motors.addActuators(configuration);

	// 只有当两者都成功添加到混控矩阵时，才返回 true
	return (mc_added_successfully && rover_added_successfully);
}

void ActuatorEffectivenessQuadRover::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	// 安全机制：当油门 (Thrust Z) 为 0 时，强制停止多旋翼的电机
	// 注意：这里只传入了 _mc_motors_mask，因此它绝不会误停漫游车的车轮！
	stopMaskedMotorsWithZeroThrust(_mc_motors_mask, actuator_sp);
}
