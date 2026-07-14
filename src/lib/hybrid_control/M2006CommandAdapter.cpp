#include "M2006CommandAdapter.hpp"

#include <cmath>

namespace hybrid_control
{

bool validM2006MotorIds(const int left_id, const int right_id)
{
	return left_id == 1 && right_id == 2;
}

M2006NormalizedCommand adaptM2006Command(const float controls[6], bool reverse_left, bool reverse_right)
{
	const float left = controls[5];
	const float right = controls[4];

	if (!std::isfinite(left) || !std::isfinite(right)) {
		return {0.f, 0.f, false};
	}

	return {reverse_left ? -left : left, reverse_right ? -right : right, true};
}

} // namespace hybrid_control
