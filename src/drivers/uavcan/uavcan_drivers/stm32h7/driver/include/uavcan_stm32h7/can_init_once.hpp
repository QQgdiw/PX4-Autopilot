#pragma once

#include <pthread.h>

namespace uavcan_stm32h7
{

struct CanInitOnce
{
	enum State {
		Uninitialized,
		Initializing,
		Initialized
	};

	pthread_mutex_t mutex;
	pthread_cond_t condition;
	State state;
	unsigned waiters;
};

#define UAVCAN_STM32H7_CAN_INIT_ONCE_INITIALIZER \
	{ PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, \
	  uavcan_stm32h7::CanInitOnce::Uninitialized, 0 }

inline bool runCanInitOnce(CanInitOnce &once, void (*const initializer)())
{
	if (pthread_mutex_lock(&once.mutex) != 0) {
		return false;
	}

	while (once.state == CanInitOnce::Initializing) {
		++once.waiters;
		const int wait_result = pthread_cond_wait(&once.condition, &once.mutex);
		--once.waiters;

		if (wait_result != 0) {
			(void)pthread_mutex_unlock(&once.mutex);
			return false;
		}
	}

	if (once.state == CanInitOnce::Initialized) {
		return pthread_mutex_unlock(&once.mutex) == 0;
	}

	once.state = CanInitOnce::Initializing;

	if (pthread_mutex_unlock(&once.mutex) != 0) {
		return false;
	}

	initializer();

	if (pthread_mutex_lock(&once.mutex) != 0) {
		return false;
	}

	once.state = CanInitOnce::Initialized;
	const int broadcast_result = once.waiters > 0 ? pthread_cond_broadcast(&once.condition) : 0;
	const int unlock_result = pthread_mutex_unlock(&once.mutex);
	return broadcast_result == 0 && unlock_result == 0;
}

#if defined(__PX4_POSIX)
inline unsigned canInitOnceWaiterCount(CanInitOnce &once)
{
	if (pthread_mutex_lock(&once.mutex) != 0) {
		return 0;
	}

	const unsigned waiters = once.waiters;
	(void)pthread_mutex_unlock(&once.mutex);
	return waiters;
}
#endif

} // namespace uavcan_stm32h7
