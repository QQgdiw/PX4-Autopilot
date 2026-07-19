#include "Hx8UartServo.hpp"

#include <cerrno>
#include <cstring>

#include <px4_platform_common/getopt.h>

int Hx8UartServo::task_spawn(int argc, char *argv[])
{
	const char *device = nullptr;
	const char *myoptarg = nullptr;
	int myoptind = 1;
	int option;

	while ((option = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		if (option == 'd') {
			device = myoptarg;
		}
	}

	if (device == nullptr) {
		return -EINVAL;
	}

	auto *instance = new Hx8UartServo(device);

	if (instance == nullptr || instance->init() != PX4_OK) {
		delete instance;
		return -EINVAL;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	return PX4_OK;
}

int Hx8UartServo::custom_command(int argc, char *argv[])
{
	Hx8UartServo *instance = _object.load();

	if (instance == nullptr || argc < 1) {
		return print_usage("driver is not running");
	}

	if (!strcmp(argv[0], "status")) {
		return instance->print_status();
	}

	if (!strcmp(argv[0], "config") && argc > 1 && !strcmp(argv[1], "check")) {
		return instance->cli_config_check();
	}

	if (!strcmp(argv[0], "config") && argc > 1 && !strcmp(argv[1], "write")) {
		return instance->cli_config_write();
	}

	return print_usage("unsupported command");
}

int Hx8UartServo::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PX4_INFO("hx8_uart_servo start -d <device>; status; config check; config write; stop");
	return PX4_OK;
}

extern "C" __EXPORT int hx8_uart_servo_main(int argc, char *argv[])
{
	return Hx8UartServo::main(argc, argv);
}
