px4_add_module(
	MODULE drivers__actuators__hx8_uart_servo
	MAIN hx8_uart_servo
	SRCS
		Hx8UartServo.cpp
		Hx8UartServo.hpp
		hx8_uart_servo_main.cpp
	DEPENDS
		hx8_protocol
		hx8_controller
		px4_work_queue
	MODULE_CONFIG module.yaml
)
