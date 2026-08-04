/****************************************************************************
 *
 *   Copyright (c) 2014-2017 PX4 Development Team. All rights reserved.
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

/**
 * @file mavlink_stream.cpp
 * Mavlink messages stream implementation.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <stdlib.h>

#include "mavlink_stream.h"
#include "mavlink_main.h"

MavlinkStream::MavlinkStream(Mavlink *mavlink) :
	_mavlink(mavlink)
{
	_operation_mutex_initialized = pthread_mutex_init(&_operation_mutex, nullptr) == 0;
	_last_sent = hrt_absolute_time();
}

MavlinkStream::~MavlinkStream()
{
	if (_operation_mutex_initialized) {
		pthread_mutex_destroy(&_operation_mutex);
	}
}

bool MavlinkStream::lock(bool nonblocking)
{
	if (!_operation_mutex_initialized) {
		return false;
	}

	const int result = nonblocking ? pthread_mutex_trylock(&_operation_mutex) : pthread_mutex_lock(&_operation_mutex);
	return result == 0;
}

void MavlinkStream::unlock()
{
	pthread_mutex_unlock(&_operation_mutex);
}

bool MavlinkStream::set_interval(int interval, bool nonblocking)
{
	if (!lock(nonblocking)) {
		return false;
	}

	_interval = interval;
	unlock();
	return true;
}

int MavlinkStream::get_interval()
{
	if (!lock(false)) {
		return 0;
	}

	const int interval = _interval;
	unlock();
	return interval;
}

bool MavlinkStream::get_rate_configuration(int &interval, unsigned &size_avg, bool &is_const_rate)
{
	if (!lock(true)) {
		return false;
	}

	interval = _interval;
	size_avg = get_size_avg();
	is_const_rate = const_rate();
	unlock();
	return true;
}

bool MavlinkStream::get_status_snapshot(int &interval, unsigned &size, bool &is_const_rate)
{
	if (!lock(true)) {
		return false;
	}

	interval = _interval;
	size = get_size();
	is_const_rate = const_rate();
	unlock();
	return true;
}

bool MavlinkStream::request_message(float param2, float param3, float param4, float param5, float param6, float param7)
{
	if (!lock(false)) {
		return false;
	}

	const bool result = request_message_impl(param2, param3, param4, param5, param6, param7);
	unlock();
	return result;
}

bool MavlinkStreamLifecycle::release_reader()
{
	if (_reader_count == 0) {
		return false;
	}

	--_reader_count;
	return true;
}

bool MavlinkStreamLifecycle::retire(List<MavlinkStream *> &active_streams, MavlinkStream *stream)
{
	if (!active_streams.remove(stream)) {
		return false;
	}

	if (_reader_count > 0) {
		_retired_streams.add(stream);

	} else {
		delete stream;
	}

	return true;
}

void MavlinkStreamLifecycle::cleanup_retired()
{
	if (_reader_count == 0) {
		_retired_streams.clear();
	}
}

void MavlinkStreamLifecycle::clear(List<MavlinkStream *> &active_streams)
{
	while (MavlinkStream *stream = active_streams.getHead()) {
		active_streams.remove(stream);

		if (_reader_count > 0) {
			_retired_streams.add(stream);

		} else {
			delete stream;
		}
	}

	cleanup_retired();
}

/**
 * Update subscriptions and send message if necessary
 */
int
MavlinkStream::update(const hrt_abstime &t)
{
	if (!lock(true)) {
		update_data_while_requesting();
		return -1;
	}

	const int result = update_unlocked(t);
	unlock();
	return result;
}

int
MavlinkStream::update_unlocked(const hrt_abstime &t)
{
	update_data();

	// If the message has never been sent before we want
	// to send it immediately and can return right away
	if (_last_sent == 0) {
		// this will give different messages on the same run a different
		// initial timestamp which will help spacing them out
		// on the link scheduling
		if (send()) {
			_last_sent = hrt_absolute_time();

			if (!_first_message_sent) {
				_first_message_sent = true;
			}
		}

		return 0;
	}

	// One of the previous iterations sent the update
	// already before the deadline
	if (_last_sent > t) {
		return -1;
	}

	int64_t dt = t - _last_sent;
	int interval = _interval;

	if (!const_rate()) {
		interval /= _mavlink->get_rate_mult();
	}

	// We don't need to send anything if the inverval is 0. send() will be called manually.
	if (interval == 0) {
		return 0;
	}

	const bool unlimited_rate = interval < 0;

	// Send the message if it is due or
	// if it will overrun the next scheduled send interval
	// by 30% of the interval time. This helps to avoid
	// sending a scheduled message on average slower than
	// scheduled. Doing this at 50% would risk sending
	// the message too often as the loop runtime of the app
	// needs to be accounted for as well.
	// This method is not theoretically optimal but a suitable
	// stopgap as it hits its deadlines well (0.5 Hz, 50 Hz and 250 Hz)

	if (unlimited_rate || (dt > (interval - (_mavlink->get_main_loop_delay() / 10) * 3))) {
		// interval expired, send message

		// If the interval is non-zero and dt is smaller than 1.5 times the interval
		// do not use the actual time but increment at a fixed rate, so that processing delays do not
		// distort the average rate. The check of the maximum interval is done to ensure that after a
		// long time not sending anything, sending multiple messages in a short time is avoided.
		if (send()) {
			_last_sent = ((interval > 0) && ((int64_t)(1.5f * interval) > dt)) ? _last_sent + interval : t;

			if (!_first_message_sent) {
				_first_message_sent = true;
			}

			return 0;

		} else {
			return -1;
		}
	}

	return -1;
}
