#include "Hx8UartServo.hpp"
#include <px4_platform_common/getopt.h>
#include <cstring>

int Hx8UartServo::task_spawn(int argc, char *argv[])
{
	const char *device = nullptr; const char *myoptarg = nullptr; int myoptind = 1; int ch;
	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) if (ch == 'd') device = myoptarg;
	if (!device) return -EINVAL;
	auto *dev = new Hx8UartServo(device); if (!dev || dev->init() != 0) { delete dev; return -EINVAL; }
	_object.store(dev); _task_id = task_id_is_work_queue; return 0;
}

int Hx8UartServo::custom_command(int argc, char *argv[])
{
	Hx8UartServo *object = _object.load();
	if (!object || argc < 1) return print_usage("driver is not running");
	if (!strcmp(argv[0], "status")) return object->print_status();
	if (!strcmp(argv[0], "config") && argc > 1 && !strcmp(argv[1], "check")) return object->cli_config_check();
	if (!strcmp(argv[0], "config") && argc > 1 && !strcmp(argv[1], "write")) return object->cli_config_write();
	return print_usage("unsupported command");
}
int Hx8UartServo::print_usage(const char *reason) { if (reason) PX4_WARN("%s", reason); PX4_INFO("hx8_uart_servo start -d <device>; status; config check; config write; stop"); return 0; }
extern "C" __EXPORT int hx8_uart_servo_main(int argc, char *argv[]) { return Hx8UartServo::main(argc, argv); }
