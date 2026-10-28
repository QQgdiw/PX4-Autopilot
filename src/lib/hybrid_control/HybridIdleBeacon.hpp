#pragma once

#include <cstdint>

namespace hybrid_control
{

// Pure timing policy. The output driver supplies actual stop state and safety gates.
// No command is queued: eligibility is re-evaluated at the instant a frame is sent.
class HybridIdleBeacon
{
public:
	bool update(uint64_t now, bool eligible, uint64_t delay, uint64_t interval)
	{
		if (!eligible || delay == 0 || interval == 0 || now == 0
		    || (_since != 0 && (now < _last_update || now - _last_update > 500000))
		    || delay != _delay || interval != _interval) {
			_since = 0;
			_last_beep = 0;
		}

		_delay = delay;
		_interval = interval;
		_last_update = now;

		if (!eligible || delay == 0 || interval == 0 || now == 0) {
			return false;
		}

		if (_since == 0) { _since = now; }

		if (now - _since < delay || (_last_beep != 0 && now - _last_beep < interval)) {
			return false;
		}

		_last_beep = now;
		return true;
	}

private:
	uint64_t _since{0};
	uint64_t _last_beep{0};
	uint64_t _last_update{0};
	uint64_t _delay{0};
	uint64_t _interval{0};
};

} // namespace hybrid_control
