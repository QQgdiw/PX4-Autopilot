px4_add_module(
	MODULE drivers__m2006_can
	MAIN m2006_can
	STACK_MAIN 2048
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}/..
		${LIBDRONECAN_DIR}/libuavcan/include
		${LIBDRONECAN_DIR_DRIVERS}/${UAVCAN_DRIVER}/driver/include
	SRCS
		M2006Can.cpp
		M2006Can.hpp
	DEPENDS
		hybrid_control
		uavcan
		uavcan_can_ownership
		uavcan_${UAVCAN_DRIVER}_driver
)
