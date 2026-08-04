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

#pragma once

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include <px4_platform_common/atomic.h>

class MavlinkStreamConfigHandoff
{
public:
	enum class Result : uint8_t {
		Success = 0,
		Failed,
		Timeout,
		Exiting,
		Invalid,
	};

	using CommitCallback = int (*)(void *context, const char *stream_name, float rate);
	using ConfigureCallback = Result(*)(void *context, const char *stream_name, float rate,
					    MavlinkStreamConfigHandoff &handoff, uint32_t generation);
	using ClockCallback = int (*)(void *context, clockid_t clock_id, timespec *time);

	/** Far above the 100 Hz main-loop handoff latency while bounding command ACK delay. */
	static constexpr uint64_t DEFAULT_TIMEOUT_US{1'000'000};
	/** Reserve time for the non-blocking stream apply section at the deadline edge. */
	static constexpr uint64_t APPLY_BUDGET_US{10'000};
	static constexpr size_t STREAM_NAME_CAPACITY{64};
	static constexpr uint32_t LIFECYCLE_CLOSING_BIT{1u << 31};
	static constexpr uint32_t LIFECYCLE_CALL_COUNT_MASK{LIFECYCLE_CLOSING_BIT - 1};

	explicit MavlinkStreamConfigHandoff(ClockCallback clock_callback = nullptr, void *clock_context = nullptr);
	/** The owner must prevent new calls before destruction; the internal gate drains admitted calls. */
	~MavlinkStreamConfigHandoff();

	MavlinkStreamConfigHandoff(const MavlinkStreamConfigHandoff &) = delete;
	MavlinkStreamConfigHandoff &operator=(const MavlinkStreamConfigHandoff &) = delete;

	Result request(const char *stream_name, float rate, uint64_t timeout_us = DEFAULT_TIMEOUT_US);

	/**
	 * Process at most one request. Must only be called from the MAVLink main thread.
	 * The callback runs outside the handoff mutex and may do bounded preparation. It must
	 * route every stream-list side effect through commit(), which atomically rejects an
	 * expired, cancelled, or superseded generation.
	 */
	bool process_pending(ConfigureCallback callback, void *context);

	/**
	 * Execute the side-effecting part of a configure callback for the matching request.
	 * The callback runs in a short commit section and must not allocate, destroy,
	 * block, log, or call this handoff. Preparation that can exceed APPLY_BUDGET_US
	 * belongs in ConfigureCallback before commit().
	 */
	Result commit(uint32_t generation, CommitCallback callback, void *context);

	/** Reject new requests and wake all current waiters. */
	void shutdown();

	bool has_pending_request();
	bool initialized() const { return _initialized; }

	static const char *result_string(Result result);
	static bool interval_to_rate(float interval_us, float &rate_hz);

private:
	enum class State : uint8_t {
		Idle = 0,
		Pending,
		Executing,
		Committing,
		Completed,
	};

	Result request_impl(const char *stream_name, float rate, uint64_t timeout_us);
	Result commit_impl(uint32_t generation, CommitCallback callback, void *context);
	bool enter();
	void leave();
	void shutdown_impl();
	int lock_until(const timespec &deadline);
	bool deadline_expired(const timespec &deadline) const;
	int read_clock(timespec &time) const;
	int wait_until(const timespec &deadline);
	void release_request_locked();

	pthread_mutex_t _mutex{};
	pthread_cond_t _condition{};
	clockid_t _condition_clock{CLOCK_MONOTONIC};
	ClockCallback _clock_callback{nullptr};
	void *_clock_context{nullptr};
	bool _mutex_initialized{false};
	bool _condition_initialized{false};
	bool _initialized{false};
	bool _shutdown{false};
	bool _admission_held{false};
	bool _request_active{false};
	bool _caller_waiting{false};
	bool _cancel_requested{false};
	px4::atomic<uint32_t> _lifecycle_state{0};
	State _state{State::Idle};
	Result _result{Result::Failed};
	uint32_t _next_generation{0};
	uint32_t _active_generation{0};
	timespec _deadline{};
	timespec _execution_deadline{};
	char _stream_name[STREAM_NAME_CAPACITY] {};
	float _rate{0.f};
};
