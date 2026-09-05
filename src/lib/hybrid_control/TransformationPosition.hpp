#pragma once

#include <cmath>
#include <cstdint>

namespace hybrid_control
{

enum class SensorSource : uint8_t { None, As5600, Tmag5273, Hx8, Hx65 };

struct PositionSample {
	float normalized{NAN};
	bool valid{false};
	bool endpoint_confirmed{false};
	SensorSource source{SensorSource::None};
	uint64_t timestamp_us{0};
};

struct TmagVector {
	float x;
	float y;
	float z;
};

float normalizeAs5600(float angle, float quad_angle, float rover_angle);
float tmagMagnitude(const TmagVector &sample);
bool tmagPairValid(bool quad_valid, bool rover_valid);

class TmagRatioFilter
{
public:
	PositionSample update(const TmagVector &quad, const TmagVector &rover,
			      bool quad_valid, bool rover_valid, bool endpoint_confirmed, uint64_t timestamp_us);
	void reset();

private:
	float _filtered{NAN};
};

enum class ProgressResult : uint8_t { Idle, Progress, NoProgress, Reached, Invalid };

class DirectedProgressMonitor
{
public:
	void start(float position, float target, uint64_t now_us);
	ProgressResult update(const PositionSample &, float target, float minimum_progress, uint64_t timeout_us);
	void reset();
	uint64_t noProgressElapsed(uint64_t now_us) const;

private:
	float _anchor{NAN};
	float _direction{0.f};
	uint64_t _anchor_time_us{0};
	bool _active{false};
};

} // namespace hybrid_control
