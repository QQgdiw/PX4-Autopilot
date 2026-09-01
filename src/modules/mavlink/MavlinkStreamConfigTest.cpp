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
#include "mavlink_stream.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>
#include <vector>

#include <px4_platform_common/defines.h>

extern "C" hrt_abstime hrt_absolute_time()
{
	return std::chrono::duration_cast<std::chrono::microseconds>(
		       std::chrono::steady_clock::now().time_since_epoch()).count();
}

namespace
{

using Result = MavlinkStreamConfigHandoff::Result;

struct RequestRecord {
	char name[MavlinkStreamConfigHandoff::STREAM_NAME_CAPACITY] {};
	float rate{0.f};
};

struct CallbackContext {
	std::vector<RequestRecord> requests;
	const char *failure_name{nullptr};
	std::atomic<unsigned> calls{0};
};

int record_commit(void *context, const char *stream_name, float rate)
{
	auto *callback_context = static_cast<CallbackContext *>(context);
	RequestRecord request{};
	strncpy(request.name, stream_name, sizeof(request.name) - 1);
	request.rate = rate;
	callback_context->requests.push_back(request);
	callback_context->calls.fetch_add(1);
	return callback_context->failure_name != nullptr && strcmp(stream_name, callback_context->failure_name) == 0 ? -1 : 0;
}

Result record_request(void *context, const char *, float, MavlinkStreamConfigHandoff &handoff, uint32_t generation)
{
	return handoff.commit(generation, record_commit, context);
}

Result success_without_commit(void *, const char *, float, MavlinkStreamConfigHandoff &, uint32_t)
{
	return Result::Success;
}

struct BlockingCallbackContext {
	std::atomic<bool> started{false};
	std::atomic<bool> release{false};
	std::atomic<unsigned> commits{0};
};

int count_commit(void *context, const char *, float)
{
	auto *callback_context = static_cast<BlockingCallbackContext *>(context);
	callback_context->commits.fetch_add(1);
	return 0;
}

Result blocking_request(void *context, const char *, float, MavlinkStreamConfigHandoff &handoff,
			uint32_t generation)
{
	auto *callback_context = static_cast<BlockingCallbackContext *>(context);
	callback_context->started.store(true);

	while (!callback_context->release.load()) {
		std::this_thread::yield();
	}

	return handoff.commit(generation, count_commit, context);
}

int blocking_commit(void *context, const char *, float)
{
	auto *callback_context = static_cast<BlockingCallbackContext *>(context);
	callback_context->started.store(true);

	while (!callback_context->release.load()) {
		std::this_thread::yield();
	}

	callback_context->commits.fetch_add(1);
	return 0;
}

Result request_blocking_commit(void *context, const char *, float, MavlinkStreamConfigHandoff &handoff,
			       uint32_t generation)
{
	return handoff.commit(generation, blocking_commit, context);
}

bool wait_for_pending(MavlinkStreamConfigHandoff &handoff)
{
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);

	while (!handoff.has_pending_request()) {
		if (std::chrono::steady_clock::now() >= deadline) {
			return false;
		}

		std::this_thread::yield();
	}

	return true;
}

Result submit_and_process(MavlinkStreamConfigHandoff &handoff, const char *name, float rate,
			  CallbackContext &callback_context)
{
	Result result{Result::Failed};
	std::thread caller([&]() { result = handoff.request(name, rate); });

	if (!wait_for_pending(handoff)) {
		handoff.shutdown();
		caller.join();
		ADD_FAILURE() << "request was not published";
		return result;
	}

	EXPECT_TRUE(handoff.process_pending(record_request, &callback_context));
	caller.join();
	return result;
}

struct TestClock {
	timespec base{};
	std::atomic<uint64_t> offset_us{0};

	TestClock()
	{
		system_clock_gettime(CLOCK_MONOTONIC, &base);
	}

	void advance(uint64_t delta_us)
	{
		offset_us.fetch_add(delta_us);
	}

	static int read(void *context, clockid_t, timespec *time)
	{
		auto *clock = static_cast<TestClock *>(context);
		const uint64_t current_offset_us = clock->offset_us.load();
		*time = clock->base;
		time->tv_sec += current_offset_us / 1'000'000;
		time->tv_nsec += static_cast<long>((current_offset_us % 1'000'000) * 1'000);

		if (time->tv_nsec >= 1'000'000'000L) {
			time->tv_sec++;
			time->tv_nsec -= 1'000'000'000L;
		}

		return 0;
	}
};

class BlockingTestStream : public MavlinkStream
{
public:
	explicit BlockingTestStream(std::atomic<unsigned> *destruction_count = nullptr) :
		MavlinkStream(nullptr),
		_destruction_count(destruction_count)
	{
		set_interval(0);
	}

	~BlockingTestStream() override
	{
		if (_destruction_count != nullptr) {
			_destruction_count->fetch_add(1);
		}
	}

	const char *get_name() const override { return "TEST"; }
	uint16_t get_id() override { return 1; }
	bool const_rate() override { return true; }
	unsigned get_size() override { return 1; }

	std::atomic<bool> request_started{false};
	std::atomic<bool> release_request{false};
	std::atomic<unsigned> update_calls{0};
	std::atomic<unsigned> deferred_update_calls{0};

protected:
	bool send() override { return true; }

	bool request_message_impl(float, float, float, float, float, float) override
	{
		request_started.store(true);

		while (!release_request.load()) {
			std::this_thread::yield();
		}

		return true;
	}

	void update_data() override { update_calls.fetch_add(1); }
	void update_data_while_requesting() override { deferred_update_calls.fetch_add(1); }

private:
	std::atomic<unsigned> *_destruction_count;
};

class LifetimeTestStream : public MavlinkStream
{
public:
	explicit LifetimeTestStream(std::atomic<unsigned> &destruction_count) :
		MavlinkStream(nullptr),
		_destruction_count(destruction_count)
	{
		set_interval(0);
	}

	~LifetimeTestStream() override { _destruction_count.fetch_add(1); }

	const char *get_name() const override { return "LIFETIME_TEST"; }
	uint16_t get_id() override { return 2; }
	unsigned get_size() override { return 1; }

protected:
	bool send() override { return true; }

private:
	std::atomic<unsigned> &_destruction_count;
};

} // namespace

TEST(MavlinkStreamConfig, TransfersExactRequestAndResult)
{
	MavlinkStreamConfigHandoff handoff;
	ASSERT_TRUE(handoff.initialized());
	CallbackContext callback_context{};

	EXPECT_EQ(submit_and_process(handoff, "GIMBAL_MANAGER_STATUS", 30.f, callback_context), Result::Success);
	ASSERT_EQ(callback_context.requests.size(), 1u);
	EXPECT_STREQ(callback_context.requests[0].name, "GIMBAL_MANAGER_STATUS");
	EXPECT_FLOAT_EQ(callback_context.requests[0].rate, 30.f);

	callback_context.failure_name = "FAIL";
	EXPECT_EQ(submit_and_process(handoff, "FAIL", 1.f, callback_context), Result::Failed);
	ASSERT_EQ(callback_context.requests.size(), 2u);
	EXPECT_STREQ(callback_context.requests[1].name, "FAIL");
}

TEST(MavlinkStreamConfig, TransfersStandardAndRoverTuningStreams)
{
	MavlinkStreamConfigHandoff handoff;
	CallbackContext callback_context{};
	const RequestRecord expected[] {
		{"GIMBAL_MANAGER_STATUS", 30.f},
		{"ROVER_RATE_TUNING_STATUS", 50.f},
		{"ROVER_ATTITUDE_TUNING_STATUS", 30.f},
		{"ROVER_VELOCITY_TUNING_STATUS", 25.f},
		{"ROVER_POSITION_TUNING_STATUS", 10.f},
	};

	for (const RequestRecord &request : expected) {
		EXPECT_EQ(submit_and_process(handoff, request.name, request.rate, callback_context), Result::Success);
	}

	ASSERT_EQ(callback_context.requests.size(), sizeof(expected) / sizeof(expected[0]));

	for (size_t index = 0; index < callback_context.requests.size(); ++index) {
		EXPECT_STREQ(callback_context.requests[index].name, expected[index].name);
		EXPECT_FLOAT_EQ(callback_context.requests[index].rate, expected[index].rate);
	}
}

TEST(MavlinkStreamConfig, PropagatesDefaultRestoreSuccessAndFailure)
{
	MavlinkStreamConfigHandoff handoff;
	CallbackContext callback_context{};

	EXPECT_EQ(submit_and_process(handoff, "RESTORE", -2.f, callback_context), Result::Success);
	callback_context.failure_name = "RESTORE_FAIL";
	EXPECT_EQ(submit_and_process(handoff, "RESTORE_FAIL", -2.f, callback_context), Result::Failed);
	ASSERT_EQ(callback_context.requests.size(), 2u);
	EXPECT_FLOAT_EQ(callback_context.requests[0].rate, -2.f);
	EXPECT_FLOAT_EQ(callback_context.requests[1].rate, -2.f);
}

TEST(MavlinkStreamConfig, RejectsSuccessWithoutExecutingCommit)
{
	MavlinkStreamConfigHandoff handoff;
	Result result{Result::Success};
	std::thread caller([&]() { result = handoff.request("NO_COMMIT", 1.f); });

	ASSERT_TRUE(wait_for_pending(handoff));
	EXPECT_TRUE(handoff.process_pending(success_without_commit, nullptr));
	caller.join();
	EXPECT_EQ(result, Result::Failed);
}

TEST(MavlinkStreamConfig, ConcurrentRequestsRemainIsolated)
{
	MavlinkStreamConfigHandoff handoff;
	CallbackContext callback_context{};
	callback_context.failure_name = "FAIL";
	std::atomic<bool> start{false};
	Result success_result{Result::Invalid};
	Result failure_result{Result::Invalid};

	std::thread success_caller([&]() {
		while (!start.load()) { std::this_thread::yield(); }

		success_result = handoff.request("SUCCESS", 50.f, 5'000'000);
	});
	std::thread failure_caller([&]() {
		while (!start.load()) { std::this_thread::yield(); }

		failure_result = handoff.request("FAIL", 10.f, 5'000'000);
	});
	start.store(true);

	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);

	while (callback_context.calls.load() < 2 && std::chrono::steady_clock::now() < deadline) {
		if (!handoff.process_pending(record_request, &callback_context)) {
			std::this_thread::yield();
		}
	}

	if (callback_context.calls.load() < 2) {
		handoff.shutdown();
	}

	success_caller.join();
	failure_caller.join();
	ASSERT_EQ(callback_context.calls.load(), 2u);
	EXPECT_EQ(success_result, Result::Success);
	EXPECT_EQ(failure_result, Result::Failed);
	ASSERT_EQ(callback_context.requests.size(), 2u);
	EXPECT_NE(strcmp(callback_context.requests[0].name, callback_context.requests[1].name), 0);
}

TEST(MavlinkStreamConfig, PreservesEnableDisableRestoreOrder)
{
	MavlinkStreamConfigHandoff handoff;
	CallbackContext callback_context{};

	EXPECT_EQ(submit_and_process(handoff, "ROVER_RATE_TUNING_STATUS", 50.f, callback_context), Result::Success);
	EXPECT_EQ(submit_and_process(handoff, "ROVER_RATE_TUNING_STATUS", 0.f, callback_context), Result::Success);
	EXPECT_EQ(submit_and_process(handoff, "ROVER_RATE_TUNING_STATUS", -2.f, callback_context), Result::Success);
	ASSERT_EQ(callback_context.requests.size(), 3u);
	EXPECT_FLOAT_EQ(callback_context.requests[0].rate, 50.f);
	EXPECT_FLOAT_EQ(callback_context.requests[1].rate, 0.f);
	EXPECT_FLOAT_EQ(callback_context.requests[2].rate, -2.f);
}

TEST(MavlinkStreamConfig, TimesOutWithoutConsumerAndRemainsUsable)
{
	MavlinkStreamConfigHandoff handoff;
	const auto start = std::chrono::steady_clock::now();
	EXPECT_EQ(handoff.request("NO_CONSUMER", 1.f, 20'000), Result::Timeout);
	const auto elapsed = std::chrono::steady_clock::now() - start;
	EXPECT_LT(elapsed, std::chrono::milliseconds(250));
	EXPECT_FALSE(handoff.has_pending_request());

	CallbackContext callback_context{};
	EXPECT_EQ(submit_and_process(handoff, "RECOVERY", 2.f, callback_context), Result::Success);
}

TEST(MavlinkStreamConfig, WaitingForSlotUsesTheSameDeadline)
{
	MavlinkStreamConfigHandoff handoff;
	Result first_result{Result::Failed};
	std::thread first_caller([&]() { first_result = handoff.request("FIRST", 1.f); });

	if (!wait_for_pending(handoff)) {
		handoff.shutdown();
		first_caller.join();
		FAIL() << "first request was not published";
	}

	const auto start = std::chrono::steady_clock::now();
	EXPECT_EQ(handoff.request("SECOND", 2.f, 20'000), Result::Timeout);
	EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(250));
	EXPECT_TRUE(handoff.has_pending_request());

	handoff.shutdown();
	first_caller.join();
	EXPECT_EQ(first_result, Result::Exiting);
}

TEST(MavlinkStreamConfig, MutexContentionUsesTheSameDeadline)
{
	MavlinkStreamConfigHandoff handoff;
	BlockingCallbackContext callback_context{};
	Result first_result{Result::Failed};
	bool request_processed = false;
	std::thread first_caller([&]() { first_result = handoff.request("FIRST", 1.f); });

	if (!wait_for_pending(handoff)) {
		handoff.shutdown();
		first_caller.join();
		FAIL() << "first request was not published";
	}

	std::thread consumer([&]() {
		request_processed = handoff.process_pending(request_blocking_commit, &callback_context);
	});
	const auto commit_start_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);

	while (!callback_context.started.load() && std::chrono::steady_clock::now() < commit_start_deadline) {
		std::this_thread::yield();
	}

	if (!callback_context.started.load()) {
		callback_context.release.store(true);
		consumer.join();
		handoff.shutdown();
		first_caller.join();
		FAIL() << "commit did not start";
	}

	const auto start = std::chrono::steady_clock::now();
	EXPECT_EQ(handoff.request("SECOND", 2.f, 20'000), Result::Timeout);
	EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(250));

	callback_context.release.store(true);
	consumer.join();
	first_caller.join();
	EXPECT_TRUE(request_processed);
	EXPECT_EQ(first_result, Result::Success);
	EXPECT_EQ(callback_context.commits.load(), 1u);
}

TEST(MavlinkStreamConfig, ExpiredRequestCannotExecuteLate)
{
	TestClock clock;
	MavlinkStreamConfigHandoff handoff(TestClock::read, &clock);
	CallbackContext callback_context{};
	Result result{Result::Failed};
	std::thread caller([&]() { result = handoff.request("EXPIRED", 1.f, 5'000'000); });

	if (!wait_for_pending(handoff)) {
		handoff.shutdown();
		caller.join();
		FAIL() << "request was not published";
	}

	clock.advance(5'000'001);
	handoff.process_pending(record_request, &callback_context);
	caller.join();
	EXPECT_EQ(result, Result::Timeout);
	EXPECT_EQ(callback_context.calls.load(), 0u);
	EXPECT_FALSE(handoff.process_pending(record_request, &callback_context));

	EXPECT_EQ(submit_and_process(handoff, "NEXT_GENERATION", 3.f, callback_context), Result::Success);
	ASSERT_EQ(callback_context.requests.size(), 1u);
	EXPECT_STREQ(callback_context.requests[0].name, "NEXT_GENERATION");
}

TEST(MavlinkStreamConfig, ApplyBudgetRejectsLateCommit)
{
	TestClock clock;
	MavlinkStreamConfigHandoff handoff(TestClock::read, &clock);
	CallbackContext callback_context{};
	Result result{Result::Failed};
	std::thread caller([&]() { result = handoff.request("LATE_APPLY", 1.f, 20'000); });

	ASSERT_TRUE(wait_for_pending(handoff));
	clock.advance(MavlinkStreamConfigHandoff::APPLY_BUDGET_US + 1);
	EXPECT_TRUE(handoff.process_pending(record_request, &callback_context));
	caller.join();

	EXPECT_EQ(result, Result::Timeout);
	EXPECT_EQ(callback_context.calls.load(), 0u);
}

TEST(MavlinkStreamConfig, ShutdownWakesPendingAndQueuedCallers)
{
	MavlinkStreamConfigHandoff handoff;
	Result pending_result{Result::Failed};
	Result queued_result{Result::Failed};
	std::thread pending_caller([&]() { pending_result = handoff.request("PENDING", 1.f); });

	if (!wait_for_pending(handoff)) {
		handoff.shutdown();
		pending_caller.join();
		FAIL() << "pending request was not published";
	}

	std::thread queued_caller([&]() { queued_result = handoff.request("QUEUED", 2.f); });

	const auto start = std::chrono::steady_clock::now();
	handoff.shutdown();
	pending_caller.join();
	queued_caller.join();
	EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(250));
	EXPECT_EQ(pending_result, Result::Exiting);
	EXPECT_EQ(queued_result, Result::Exiting);
	EXPECT_EQ(handoff.request("AFTER_EXIT", 1.f), Result::Exiting);
}

TEST(MavlinkStreamConfig, CompletedResultSurvivesShutdown)
{
	for (unsigned iteration = 0; iteration < 50; ++iteration) {
		MavlinkStreamConfigHandoff handoff;
		CallbackContext callback_context{};
		Result result{Result::Failed};
		std::thread caller([&]() { result = handoff.request("COMPLETED", 1.f); });

		if (!wait_for_pending(handoff)) {
			handoff.shutdown();
			caller.join();
			FAIL() << "request was not published";
		}

		ASSERT_TRUE(handoff.process_pending(record_request, &callback_context));
		handoff.shutdown();
		caller.join();
		EXPECT_EQ(result, Result::Success);
	}
}

TEST(MavlinkStreamConfig, ShutdownCompletesBeforeDestructionWithActiveCaller)
{
	auto *handoff = new MavlinkStreamConfigHandoff();
	ASSERT_NE(handoff, nullptr);
	Result result{Result::Failed};
	std::thread caller([&]() { result = handoff->request("PENDING", 1.f); });

	if (!wait_for_pending(*handoff)) {
		handoff->shutdown();
		caller.join();
		delete handoff;
		FAIL() << "request was not published";
	}

	/* The owner must cancel and join users before destroying the object. */
	handoff->shutdown();
	caller.join();
	delete handoff;
	EXPECT_EQ(result, Result::Exiting);
}

TEST(MavlinkStreamConfig, ShutdownCompletesBeforeDestructionWithExecutingConsumer)
{
	auto *handoff = new MavlinkStreamConfigHandoff();
	ASSERT_NE(handoff, nullptr);
	BlockingCallbackContext callback_context{};
	Result result{Result::Failed};
	std::thread caller([&]() { result = handoff->request("EXECUTING", 1.f); });

	if (!wait_for_pending(*handoff)) {
		handoff->shutdown();
		caller.join();
		delete handoff;
		FAIL() << "request was not published";
	}

	std::thread consumer([&]() { EXPECT_TRUE(handoff->process_pending(blocking_request, &callback_context)); });

	while (!callback_context.started.load()) {
		std::this_thread::yield();
	}

	/* Shutdown is the lifecycle barrier; destruction follows all users joining. */
	handoff->shutdown();
	callback_context.release.store(true);
	consumer.join();
	caller.join();
	delete handoff;
	EXPECT_EQ(result, Result::Exiting);
	EXPECT_EQ(callback_context.commits.load(), 0u);
}

TEST(MavlinkStreamConfig, CallbackCrossingDeadlineCannotCommitLateOrPoisonNextRequest)
{
	MavlinkStreamConfigHandoff handoff;
	BlockingCallbackContext callback_context{};
	Result result{Result::Failed};
	std::atomic<bool> caller_finished{false};
	std::thread caller([&]() {
		result = handoff.request("SLOW", 1.f, 20'000);
		caller_finished.store(true);
	});

	ASSERT_TRUE(wait_for_pending(handoff));
	std::thread consumer([&]() {
		EXPECT_TRUE(handoff.process_pending(blocking_request, &callback_context));
	});

	while (!callback_context.started.load()) {
		std::this_thread::yield();
	}

	const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);

	while (!caller_finished.load() && std::chrono::steady_clock::now() < deadline) {
		std::this_thread::yield();
	}

	const bool finished_before_release = caller_finished.load();
	EXPECT_TRUE(finished_before_release);
	callback_context.release.store(true);
	consumer.join();
	caller.join();
	EXPECT_EQ(result, Result::Timeout);
	EXPECT_EQ(callback_context.commits.load(), 0u);

	CallbackContext next_context{};
	EXPECT_EQ(submit_and_process(handoff, "AFTER_SLOW", 2.f, next_context), Result::Success);
	ASSERT_EQ(next_context.requests.size(), 1u);
	EXPECT_STREQ(next_context.requests[0].name, "AFTER_SLOW");
}

TEST(MavlinkStreamConfig, RejectsInvalidStorageAndRate)
{
	MavlinkStreamConfigHandoff handoff;
	char oversized_name[MavlinkStreamConfigHandoff::STREAM_NAME_CAPACITY + 1] {};
	memset(oversized_name, 'A', sizeof(oversized_name) - 1);

	EXPECT_EQ(handoff.request(nullptr, 1.f), Result::Invalid);
	EXPECT_EQ(handoff.request("", 1.f), Result::Invalid);
	EXPECT_EQ(handoff.request(oversized_name, 1.f), Result::Invalid);
	EXPECT_EQ(handoff.request("NAN", NAN), Result::Invalid);
	EXPECT_EQ(handoff.request("INF", INFINITY), Result::Invalid);
}

TEST(MavlinkStreamConfig, ConvertsMessageIntervals)
{
	float rate = NAN;
	ASSERT_TRUE(MavlinkStreamConfigHandoff::interval_to_rate(20'000.f, rate));
	EXPECT_FLOAT_EQ(rate, 50.f);
	ASSERT_TRUE(MavlinkStreamConfigHandoff::interval_to_rate(33'333.f, rate));
	EXPECT_NEAR(rate, 30.f, 0.001f);
	ASSERT_TRUE(MavlinkStreamConfigHandoff::interval_to_rate(40'000.f, rate));
	EXPECT_FLOAT_EQ(rate, 25.f);
	ASSERT_TRUE(MavlinkStreamConfigHandoff::interval_to_rate(100'000.f, rate));
	EXPECT_FLOAT_EQ(rate, 10.f);
	ASSERT_TRUE(MavlinkStreamConfigHandoff::interval_to_rate(-1.f, rate));
	EXPECT_FLOAT_EQ(rate, 0.f);
	ASSERT_TRUE(MavlinkStreamConfigHandoff::interval_to_rate(-0.000001f, rate));
	EXPECT_FLOAT_EQ(rate, 0.f);
	ASSERT_TRUE(MavlinkStreamConfigHandoff::interval_to_rate(0.f, rate));
	EXPECT_FLOAT_EQ(rate, -2.f);
	EXPECT_FALSE(MavlinkStreamConfigHandoff::interval_to_rate(NAN, rate));
	EXPECT_FALSE(MavlinkStreamConfigHandoff::interval_to_rate(INFINITY, rate));
}

TEST(MavlinkStreamConfig, SerializesOnlyTheRequestedStream)
{
	BlockingTestStream requested_stream;
	BlockingTestStream independent_stream;
	bool request_result = false;
	std::thread requester([&]() { request_result = requested_stream.request_message(); });
	const auto start_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);

	while (!requested_stream.request_started.load() && std::chrono::steady_clock::now() < start_deadline) {
		std::this_thread::yield();
	}

	if (!requested_stream.request_started.load()) {
		requested_stream.release_request.store(true);
		requester.join();
		FAIL() << "request_message did not start";
	}

	EXPECT_EQ(requested_stream.update(hrt_absolute_time()), -1);
	EXPECT_FALSE(requested_stream.set_interval(1'000, true));
	EXPECT_EQ(independent_stream.update(hrt_absolute_time()), 0);
	EXPECT_EQ(requested_stream.update_calls.load(), 0u);
	EXPECT_EQ(requested_stream.deferred_update_calls.load(), 1u);
	EXPECT_EQ(independent_stream.update_calls.load(), 1u);

	requested_stream.release_request.store(true);
	requester.join();
	EXPECT_TRUE(request_result);
	EXPECT_TRUE(requested_stream.set_interval(1'000, true));
}

TEST(MavlinkStreamConfig, DefersRetiredStreamDestructionUntilReaderReleases)
{
	std::atomic<unsigned> destruction_count{0};
	List<MavlinkStream *> active_streams;
	MavlinkStreamLifecycle lifecycle;
	auto *stream = new BlockingTestStream(&destruction_count);
	active_streams.add(stream);

	lifecycle.acquire_reader();
	bool request_result = false;
	std::thread requester([&]() { request_result = stream->request_message(); });
	const auto start_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);

	while (!stream->request_started.load() && std::chrono::steady_clock::now() < start_deadline) {
		std::this_thread::yield();
	}

	if (!stream->request_started.load()) {
		stream->release_request.store(true);
		requester.join();
		lifecycle.release_reader();
		lifecycle.clear(active_streams);
		FAIL() << "request_message did not start";
	}

	EXPECT_TRUE(lifecycle.retire(active_streams, stream));
	EXPECT_TRUE(active_streams.empty());
	EXPECT_EQ(lifecycle.reader_count(), 1u);
	EXPECT_EQ(lifecycle.retired_count(), 1u);
	lifecycle.cleanup_retired();
	EXPECT_EQ(destruction_count.load(), 0u);

	stream->release_request.store(true);
	requester.join();
	EXPECT_TRUE(request_result);
	EXPECT_TRUE(lifecycle.release_reader());
	lifecycle.cleanup_retired();
	EXPECT_EQ(lifecycle.retired_count(), 0u);
	EXPECT_EQ(destruction_count.load(), 1u);
	EXPECT_FALSE(lifecycle.release_reader());
}

TEST(MavlinkStreamConfig, DefersRetiredStreamDestructionUntilCleanup)
{
	std::atomic<unsigned> destruction_count{0};
	List<MavlinkStream *> active_streams;
	MavlinkStreamLifecycle lifecycle;
	auto *stream = new LifetimeTestStream(destruction_count);
	active_streams.add(stream);

	EXPECT_TRUE(lifecycle.retire(active_streams, stream));
	EXPECT_TRUE(active_streams.empty());
	EXPECT_EQ(destruction_count.load(), 0u);

	lifecycle.cleanup_retired();
	EXPECT_EQ(lifecycle.retired_count(), 0u);
	EXPECT_EQ(destruction_count.load(), 1u);
}

TEST(MavlinkStreamConfig, ShutdownClearDefersAllStreamsForAnActiveReader)
{
	std::atomic<unsigned> destruction_count{0};
	List<MavlinkStream *> active_streams;
	MavlinkStreamLifecycle lifecycle;
	active_streams.add(new LifetimeTestStream(destruction_count));
	active_streams.add(new LifetimeTestStream(destruction_count));

	lifecycle.acquire_reader();
	lifecycle.clear(active_streams);
	EXPECT_TRUE(active_streams.empty());
	EXPECT_EQ(lifecycle.retired_count(), 2u);
	EXPECT_EQ(destruction_count.load(), 0u);

	EXPECT_TRUE(lifecycle.release_reader());
	lifecycle.cleanup_retired();
	EXPECT_EQ(lifecycle.retired_count(), 0u);
	EXPECT_EQ(destruction_count.load(), 2u);
}
