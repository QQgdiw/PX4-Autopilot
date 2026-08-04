/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "mavlink_stream_config.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <px4_platform_common/defines.h>

namespace
{

void add_timeout(timespec &deadline, uint64_t timeout_us)
{
	deadline.tv_sec += timeout_us / 1'000'000;
	deadline.tv_nsec += static_cast<long>((timeout_us % 1'000'000) * 1'000);

	if (deadline.tv_nsec >= 1'000'000'000L) {
		deadline.tv_sec++;
		deadline.tv_nsec -= 1'000'000'000L;
	}
}

bool timespec_at_or_after(const timespec &lhs, const timespec &rhs)
{
	return lhs.tv_sec > rhs.tv_sec || (lhs.tv_sec == rhs.tv_sec && lhs.tv_nsec >= rhs.tv_nsec);
}

uint64_t timespec_remaining_us(const timespec &deadline, const timespec &now)
{
	if (timespec_at_or_after(now, deadline)) {
		return 0;
	}

	uint64_t seconds = static_cast<uint64_t>(deadline.tv_sec - now.tv_sec);
	long nanoseconds = deadline.tv_nsec - now.tv_nsec;

	if (nanoseconds < 0) {
		--seconds;
		nanoseconds += 1'000'000'000L;
	}

	return seconds * 1'000'000ULL + (static_cast<uint64_t>(nanoseconds) + 999ULL) / 1'000ULL;
}

} // namespace

MavlinkStreamConfigHandoff::MavlinkStreamConfigHandoff(ClockCallback clock_callback, void *clock_context) :
	_clock_callback(clock_callback),
	_clock_context(clock_context)
{
	if (pthread_mutex_init(&_mutex, nullptr) != 0) {
		return;
	}

	_mutex_initialized = true;
	pthread_condattr_t condition_attributes{};

	if (pthread_condattr_init(&condition_attributes) != 0) {
		return;
	}

#if defined(__PX4_DARWIN)
	// macOS condition variables cannot select CLOCK_MONOTONIC. wait_until()
	// converts the monotonic deadline to a relative wait instead.
#else

	if (pthread_condattr_setclock(&condition_attributes, CLOCK_MONOTONIC) != 0) {
		pthread_condattr_destroy(&condition_attributes);
		return;
	}

#endif

	if (pthread_cond_init(&_condition, &condition_attributes) == 0) {
		_condition_initialized = true;
	}

	pthread_condattr_destroy(&condition_attributes);
	_initialized = _mutex_initialized && _condition_initialized;
}

MavlinkStreamConfigHandoff::~MavlinkStreamConfigHandoff()
{
	_lifecycle_state.fetch_or(LIFECYCLE_CLOSING_BIT);
	shutdown_impl();

	/* The owner prevents new calls before destruction. Drain calls that were
	 * already admitted before destroying the synchronization objects. */
	const timespec drain_pause{0, 100'000L};

	while ((_lifecycle_state.load() & LIFECYCLE_CALL_COUNT_MASK) > 0) {
		nanosleep(&drain_pause, nullptr);
	}

	if (_condition_initialized) {
		pthread_cond_destroy(&_condition);
	}

	if (_mutex_initialized) {
		pthread_mutex_destroy(&_mutex);
	}
}

MavlinkStreamConfigHandoff::Result MavlinkStreamConfigHandoff::request(const char *stream_name, float rate,
		uint64_t timeout_us)
{
	if (!enter()) {
		return Result::Exiting;
	}

	const Result result = request_impl(stream_name, rate, timeout_us);
	leave();
	return result;
}

MavlinkStreamConfigHandoff::Result MavlinkStreamConfigHandoff::request_impl(const char *stream_name, float rate,
		uint64_t timeout_us)
{
	timespec deadline{};

	if (!_initialized || read_clock(deadline) != 0) {
		return Result::Failed;
	}

	add_timeout(deadline, timeout_us);

	if (stream_name == nullptr || stream_name[0] == '\0' || !PX4_ISFINITE(rate)) {
		return Result::Invalid;
	}

	const size_t stream_name_length = strnlen(stream_name, STREAM_NAME_CAPACITY);

	if (stream_name_length >= STREAM_NAME_CAPACITY) {
		return Result::Invalid;
	}

	const int lock_result = lock_until(deadline);

	if (lock_result != 0) {
		return lock_result == ETIMEDOUT ? Result::Timeout : Result::Failed;
	}

	while (_admission_held && !_shutdown) {
		const int wait_result = wait_until(deadline);

		if (!_admission_held || _shutdown) {
			break;
		}

		if (wait_result == ETIMEDOUT || deadline_expired(deadline)) {
			pthread_mutex_unlock(&_mutex);
			return Result::Timeout;
		}

		if (wait_result != 0) {
			pthread_mutex_unlock(&_mutex);
			return Result::Failed;
		}
	}

	if (_shutdown) {
		pthread_mutex_unlock(&_mutex);
		return Result::Exiting;
	}

	if (deadline_expired(deadline)) {
		pthread_mutex_unlock(&_mutex);
		return Result::Timeout;
	}

	_admission_held = true;
	_request_active = true;
	_caller_waiting = true;
	_cancel_requested = false;
	_state = State::Pending;
	_result = Result::Failed;
	_active_generation = ++_next_generation;
	_deadline = deadline;
	_rate = rate;
	memcpy(_stream_name, stream_name, stream_name_length + 1);
	pthread_cond_broadcast(&_condition);

	while (_request_active && _state != State::Completed) {
		const int wait_result = wait_until(deadline);

		if (!_request_active || _state == State::Completed) {
			break;
		}

		if (_shutdown) {
			_cancel_requested = true;
			_result = Result::Exiting;
			_state = State::Completed;
			const Result result = _result;
			_caller_waiting = false;
			release_request_locked();
			pthread_mutex_unlock(&_mutex);
			return result;
		}

		if (wait_result == ETIMEDOUT || deadline_expired(deadline)) {
			_cancel_requested = true;
			_result = Result::Timeout;
			_state = State::Completed;
			const Result result = _result;
			_caller_waiting = false;
			release_request_locked();
			pthread_mutex_unlock(&_mutex);
			return result;
		}

		if (wait_result != 0) {
			_cancel_requested = true;
			_result = Result::Failed;
			_state = State::Completed;
			const Result result = _result;
			_caller_waiting = false;
			release_request_locked();
			pthread_mutex_unlock(&_mutex);
			return result;
		}
	}

	const Result result = _result;
	_caller_waiting = false;
	release_request_locked();
	pthread_mutex_unlock(&_mutex);
	return result;
}

bool MavlinkStreamConfigHandoff::enter()
{
	uint32_t state = _lifecycle_state.load();

	while ((state & LIFECYCLE_CLOSING_BIT) == 0) {
		if ((state & LIFECYCLE_CALL_COUNT_MASK) == LIFECYCLE_CALL_COUNT_MASK) {
			return false;
		}

		if (_lifecycle_state.compare_exchange(&state, state + 1)) {
			return true;
		}
	}

	return false;
}

void MavlinkStreamConfigHandoff::leave()
{
	_lifecycle_state.fetch_sub(1);
}

bool MavlinkStreamConfigHandoff::process_pending(ConfigureCallback callback, void *context)
{
	if (!enter()) {
		return false;
	}

	if (!_initialized || pthread_mutex_lock(&_mutex) != 0) {
		leave();
		return false;
	}

	if (_shutdown || !_request_active || _state != State::Pending) {
		pthread_mutex_unlock(&_mutex);
		leave();
		return false;
	}

	if (deadline_expired(_deadline)) {
		_result = Result::Timeout;
		_state = State::Completed;

		if (!_caller_waiting) {
			release_request_locked();
		}

		pthread_cond_broadcast(&_condition);
		pthread_mutex_unlock(&_mutex);
		leave();
		return true;
	}

	const uint32_t generation = _active_generation;
	char stream_name[STREAM_NAME_CAPACITY] {};
	strncpy(stream_name, _stream_name, sizeof(stream_name) - 1);
	const float rate = _rate;

	if (_cancel_requested || _shutdown || deadline_expired(_deadline)) {
		_result = _shutdown ? Result::Exiting : Result::Timeout;
		_state = State::Completed;

		if (!_caller_waiting) {
			release_request_locked();
		}

		pthread_cond_broadcast(&_condition);
		pthread_mutex_unlock(&_mutex);
		leave();
		return true;
	}

	_state = State::Executing;
	pthread_mutex_unlock(&_mutex);

	/* Preparation runs without the handoff mutex. Any side effect must use commit(),
	 * where timeout/shutdown and this generation are atomically rechecked. */
	const Result callback_result = callback != nullptr ?
				       callback(context, stream_name, rate, *this, generation) : Result::Failed;

	if (pthread_mutex_lock(&_mutex) != 0) {
		leave();
		return false;
	}

	if (_request_active && _active_generation == generation && _state == State::Executing) {
		/* Success is only legal after commit() has executed the real operation. */
		_result = callback_result == Result::Success ? Result::Failed : callback_result;
		_state = State::Completed;

		if (!_caller_waiting) {
			release_request_locked();
		}

		pthread_cond_broadcast(&_condition);
	}

	pthread_mutex_unlock(&_mutex);
	leave();
	return true;
}

MavlinkStreamConfigHandoff::Result MavlinkStreamConfigHandoff::commit(uint32_t generation,
		CommitCallback callback, void *context)
{
	if (!enter()) {
		return Result::Exiting;
	}

	const Result result = commit_impl(generation, callback, context);
	leave();
	return result;
}

MavlinkStreamConfigHandoff::Result MavlinkStreamConfigHandoff::commit_impl(uint32_t generation,
		CommitCallback callback, void *context)
{
	if (!_initialized || callback == nullptr || pthread_mutex_lock(&_mutex) != 0) {
		return Result::Failed;
	}

	if (!_request_active || _active_generation != generation || _state != State::Executing) {
		const Result result = _shutdown ? Result::Exiting :
				      (_cancel_requested ? Result::Timeout : Result::Failed);
		pthread_mutex_unlock(&_mutex);
		return result;
	}

	if (_shutdown || _cancel_requested || deadline_expired(_deadline)) {
		_result = _shutdown ? Result::Exiting : Result::Timeout;
		_state = State::Completed;
		const Result result = _result;

		if (!_caller_waiting) {
			release_request_locked();
		}

		pthread_cond_broadcast(&_condition);
		pthread_mutex_unlock(&_mutex);
		return result;
	}

	_state = State::Committing;

	/* This is the linearization point for stream-list mutation. Holding the
	 * handoff mutex prevents a concurrent timeout from returning before the
	 * bounded commit reports its real result. */
	const int configure_result = callback(context, _stream_name, _rate);
	_result = configure_result == 0 ? Result::Success : Result::Failed;
	_state = State::Completed;
	const Result result = _result;

	if (!_caller_waiting) {
		release_request_locked();
	}

	pthread_cond_broadcast(&_condition);
	pthread_mutex_unlock(&_mutex);
	return result;
}

void MavlinkStreamConfigHandoff::shutdown()
{
	if (!enter()) {
		return;
	}

	shutdown_impl();
	leave();
}

void MavlinkStreamConfigHandoff::shutdown_impl()
{
	if (!_initialized || pthread_mutex_lock(&_mutex) != 0) {
		return;
	}

	_shutdown = true;

	if (_request_active && (_state == State::Pending || _state == State::Executing)) {
		_cancel_requested = true;
		_result = Result::Exiting;
		_state = State::Completed;

		if (!_caller_waiting) {
			release_request_locked();
		}
	}

	pthread_cond_broadcast(&_condition);
	pthread_mutex_unlock(&_mutex);
}

bool MavlinkStreamConfigHandoff::has_pending_request()
{
	if (!enter()) {
		return false;
	}

	if (!_initialized || pthread_mutex_lock(&_mutex) != 0) {
		leave();
		return false;
	}

	const bool pending = _request_active && _state == State::Pending;
	pthread_mutex_unlock(&_mutex);
	leave();
	return pending;
}

int MavlinkStreamConfigHandoff::lock_until(const timespec &deadline)
{
	static constexpr uint64_t MUTEX_WAIT_SLICE_US{1'000};

	while (true) {
		const int lock_result = pthread_mutex_trylock(&_mutex);

		if (lock_result == 0) {
			return 0;
		}

		if (lock_result != EBUSY) {
			return lock_result;
		}

		timespec monotonic_now{};

		if (read_clock(monotonic_now) != 0) {
			return EINVAL;
		}

		const uint64_t remaining_us = timespec_remaining_us(deadline, monotonic_now);

		if (remaining_us == 0) {
			return ETIMEDOUT;
		}

		// Block the higher-priority receiver so a lower-priority NuttX mutex owner can run.
		const uint64_t pause_us = remaining_us < MUTEX_WAIT_SLICE_US ? remaining_us : MUTEX_WAIT_SLICE_US;
		const timespec pause{static_cast<time_t>(pause_us / 1'000'000),
				     static_cast<long>((pause_us % 1'000'000) * 1'000)};
		nanosleep(&pause, nullptr);
	}
}

const char *MavlinkStreamConfigHandoff::result_string(Result result)
{
	switch (result) {
	case Result::Success:
		return "success";

	case Result::Failed:
		return "failed";

	case Result::Timeout:
		return "timed out";

	case Result::Exiting:
		return "cancelled during shutdown";

	case Result::Invalid:
		return "invalid request";
	}

	return "unknown";
}

bool MavlinkStreamConfigHandoff::interval_to_rate(float interval_us, float &rate_hz)
{
	if (!PX4_ISFINITE(interval_us)) {
		return false;
	}

	if (interval_us < 0.f) {
		rate_hz = 0.f;

	} else if (interval_us > 0.f) {
		rate_hz = 1'000'000.f / interval_us;

	} else {
		rate_hz = -2.f;
	}

	return PX4_ISFINITE(rate_hz);
}

bool MavlinkStreamConfigHandoff::deadline_expired(const timespec &deadline) const
{
	timespec now{};
	return read_clock(now) != 0 || timespec_at_or_after(now, deadline);
}

int MavlinkStreamConfigHandoff::read_clock(timespec &time) const
{
	if (_clock_callback != nullptr) {
		return _clock_callback(_clock_context, _condition_clock, &time);
	}

	return system_clock_gettime(_condition_clock, &time);
}

int MavlinkStreamConfigHandoff::wait_until(const timespec &deadline)
{
	int wait_result = 0;

#if defined(__PX4_DARWIN)
	timespec now {};

	if (read_clock(now) != 0 || timespec_at_or_after(now, deadline)) {
		return ETIMEDOUT;
	}

	timespec remaining{};
	remaining.tv_sec = deadline.tv_sec - now.tv_sec;
	remaining.tv_nsec = deadline.tv_nsec - now.tv_nsec;

	if (remaining.tv_nsec < 0) {
		--remaining.tv_sec;
		remaining.tv_nsec += 1'000'000'000L;
	}

	do {
		wait_result = pthread_cond_timedwait_relative_np(&_condition, &_mutex, &remaining);
	} while (wait_result == EINTR);

#else

	do {
		wait_result = system_pthread_cond_timedwait(&_condition, &_mutex, &deadline);
	} while (wait_result == EINTR);

#endif

	return wait_result;
}

void MavlinkStreamConfigHandoff::release_request_locked()
{
	_state = State::Idle;
	_request_active = false;
	_admission_held = false;
	_caller_waiting = false;
	_cancel_requested = false;
	_stream_name[0] = '\0';
	_rate = 0.f;
	pthread_cond_broadcast(&_condition);
}
