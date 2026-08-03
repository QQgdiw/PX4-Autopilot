px4_add_module(
	MODULE modules__mini_vehicle_control
	MAIN mini_vehicle_control
	SRCS
		mini_vehicle_control.cpp
)

px4_add_unit_gtest(
	SRC
		MiniVehicleControlDiagnosticsTest.cpp
)
