#include "TransformationPosition.hpp"

namespace hybrid_control
{

namespace
{
constexpr float kMinimumTmagMagnitude = 1e-6f;
constexpr float kTmagFilterAlpha = 0.25f;

float clamp01(float value)
{
	return fmaxf(0.f, fminf(value, 1.f));
}

float wrappedDifference(float value, float reference)
{
	const float raw = value - reference;
	const float wrapped = atan2f(sinf(raw), cosf(raw));

	// atan2f() has an implementation-dependent sign for the exact pi tie.
	// Preserve the configured endpoint direction so a 180-degree HX8 span
	// does not invert and classify the Rover endpoint as Quad.
	if (fabsf(fabsf(wrapped) - static_cast<float>(M_PI)) < 1e-5f) {
		return raw < 0.f ? -static_cast<float>(M_PI) : static_cast<float>(M_PI);
	}

	return wrapped;
}

bool finiteVector(const TmagVector &sample)
{
	return std::isfinite(sample.x) && std::isfinite(sample.y) && std::isfinite(sample.z);
}
}

float normalizeAs5600(float angle, float quad_angle, float rover_angle)
{
	if (!std::isfinite(angle) || !std::isfinite(quad_angle) || !std::isfinite(rover_angle)) {
		return NAN;
	}

	const float endpoint_travel = wrappedDifference(rover_angle, quad_angle);

	if (fabsf(endpoint_travel) < kMinimumTmagMagnitude) {
		return NAN;
	}

	// Keep the measured angle on the same directed revolution as the
	// configured endpoint travel.  A shortest-angle wrap alone is ambiguous
	// when the configured span is 180 degrees: for example, with -85 -> 95,
	// a measured 95.1 degrees is 180.1 degrees from Quad and must remain just
	// beyond the Rover endpoint, rather than wrapping to -179.9 degrees and
	// being misclassified as Quad.
	const float raw_difference = angle - quad_angle;
	const float revolutions = (endpoint_travel - raw_difference) / (2.f * static_cast<float>(M_PI));
	const float directed_difference = raw_difference
			+ roundf(revolutions) * (2.f * static_cast<float>(M_PI));
	const float position = directed_difference / endpoint_travel;
	return clamp01(position);
}

bool tmagPairValid(bool quad_valid, bool rover_valid)
{
	return quad_valid && rover_valid;
}

float tmagMagnitude(const TmagVector &sample)
{
	return sqrtf(sample.x * sample.x + sample.y * sample.y + sample.z * sample.z);
}

PositionSample TmagRatioFilter::update(const TmagVector &quad, const TmagVector &rover,
		bool quad_valid, bool rover_valid, bool endpoint_confirmed, uint64_t timestamp_us)
{
	PositionSample result{NAN, false, false, SensorSource::Tmag5273, timestamp_us};

	if (!quad_valid || !rover_valid || !finiteVector(quad) || !finiteVector(rover)) {
		return result;
	}

	const float quad_magnitude = tmagMagnitude(quad);
	const float rover_magnitude = tmagMagnitude(rover);
	const float denominator = quad_magnitude + rover_magnitude;

	if (!std::isfinite(denominator) || denominator < kMinimumTmagMagnitude) {
		return result;
	}

	const float ratio = clamp01(rover_magnitude / denominator);
	_filtered = std::isfinite(_filtered) ? _filtered + kTmagFilterAlpha * (ratio - _filtered) : ratio;
	result.normalized = _filtered;
	result.valid = true;
	result.endpoint_confirmed = endpoint_confirmed;
	return result;
}

void TmagRatioFilter::reset()
{
	_filtered = NAN;
}

void DirectedProgressMonitor::start(float position, float target, uint64_t now_us)
{
	_anchor = position;
	_direction = target > position ? 1.f : (target < position ? -1.f : 0.f);
	_anchor_time_us = now_us;
	_active = std::isfinite(position) && std::isfinite(target) && fabsf(_direction) > 0.f;
}

ProgressResult DirectedProgressMonitor::update(const PositionSample &sample, float target,
		float minimum_progress, uint64_t timeout_us)
{
	if (!_active) {
		return ProgressResult::Idle;
	}

	if (!sample.valid || !std::isfinite(sample.normalized) || !std::isfinite(target)
	    || !std::isfinite(minimum_progress) || minimum_progress <= 0.f
	    || (target - _anchor) * _direction < 0.f) {
		return ProgressResult::Invalid;
	}

	if (sample.endpoint_confirmed) {
		reset();
		return ProgressResult::Reached;
	}

	const float directed_progress = (sample.normalized - _anchor) * _direction;

	if (directed_progress >= minimum_progress) {
		_anchor = sample.normalized;
		_anchor_time_us = sample.timestamp_us;
		return ProgressResult::Progress;
	}

	if (noProgressElapsed(sample.timestamp_us) >= timeout_us) {
		return ProgressResult::NoProgress;
	}

	return ProgressResult::Progress;
}

void DirectedProgressMonitor::reset()
{
	_anchor = NAN;
	_direction = 0.f;
	_anchor_time_us = 0;
	_active = false;
}

uint64_t DirectedProgressMonitor::noProgressElapsed(uint64_t now_us) const
{
	return _active && now_us > _anchor_time_us ? now_us - _anchor_time_us : 0;
}

} // namespace hybrid_control
