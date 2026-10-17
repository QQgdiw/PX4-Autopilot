px4_add_module(
	MODULE modules__hybrid_vehicle_control
	MAIN hybrid_vehicle_control
	SRCS
		hybrid_vehicle_control.cpp
	DEPENDS
		hybrid_control
	)

px4_add_functional_gtest(SRC HybridOutputTest.cpp LINKLIBS modules__hybrid_vehicle_control hybrid_control)
if(BUILD_TESTING)
	add_dependencies(functional-HybridOutput parameters_header uorb_headers)
	target_compile_definitions(functional-HybridOutput PRIVATE MODULE_NAME="hybrid_output_test")
endif()
