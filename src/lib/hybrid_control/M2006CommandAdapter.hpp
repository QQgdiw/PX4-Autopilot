#pragma once

namespace hybrid_control
{

struct M2006NormalizedCommand {
	float left;
	float right;
	bool finite;
};

M2006NormalizedCommand adaptM2006Command(const float controls[6], bool reverse_left, bool reverse_right);
bool validM2006MotorIds(int left_id, int right_id);

} // namespace hybrid_control
