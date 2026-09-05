#include "Hx8UartServo.hpp"
#include "Hx8CliPolicy.hpp"

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

	if (argc < 1) {
		print_usage("missing command");
		return -EINVAL;
	}

	const int instance_status = hx8_cli::driverInstanceStatus(instance != nullptr);

	if (instance_status != 0) {
		print_usage("driver is not running");
		return instance_status;
	}

	if (!strcmp(argv[0], "status")) {
		return instance->print_status();
	}

	if (!strcmp(argv[0], "trace")) {
		return instance->cli_trace();
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

	PX4_INFO("hx8_uart_servo start -d <device>; status; trace; config check; config write; stop");
	return PX4_OK;
}

extern "C" __EXPORT int hx8_uart_servo_main(int argc, char *argv[])
{
	return Hx8UartServo::main(argc, argv);
}
