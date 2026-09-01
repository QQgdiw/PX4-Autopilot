px4_add_library(hybrid_control
	C610Protocol.cpp
	M2006CommandAdapter.cpp
	M2006DriveGate.cpp
	M2006SpeedController.cpp
	TransformationPosition.cpp
	TransformationStateMachine.cpp
	HybridTransitionPolicy.cpp
	Hx8BackendPolicy.cpp
)

target_include_directories(hybrid_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

px4_add_unit_gtest(SRC C610ProtocolTest.cpp LINKLIBS hybrid_control)
px4_add_unit_gtest(SRC M2006CommandAdapterTest.cpp LINKLIBS hybrid_control)
px4_add_unit_gtest(SRC M2006SpeedControllerTest.cpp LINKLIBS hybrid_control)
px4_add_unit_gtest(SRC M2006DriveGateTest.cpp LINKLIBS hybrid_control)
px4_add_unit_gtest(SRC TransformationStateMachineTest.cpp LINKLIBS hybrid_control)
px4_add_unit_gtest(SRC TransformationPositionTest.cpp LINKLIBS hybrid_control)
px4_add_unit_gtest(SRC Hx8BackendPolicyTest.cpp LINKLIBS hybrid_control)
px4_add_unit_gtest(SRC M2006TxPolicyTest.cpp LINKLIBS hybrid_control)
px4_add_unit_gtest(SRC HybridTransitionPolicyTest.cpp LINKLIBS hybrid_control)
px4_add_unit_gtest(
	SRC ${PX4_SOURCE_DIR}/src/drivers/uavcan/CanOwnershipTest.cpp
	EXTRA_SRCS ${PX4_SOURCE_DIR}/src/drivers/uavcan/CanOwnership.cpp
)
px4_add_unit_gtest(
	SRC BusOffCleanupTest.cpp
	INCLUDES
		${PX4_SOURCE_DIR}/src/drivers/uavcan/uavcan_drivers/stm32h7/driver/include
)
px4_add_unit_gtest(
	SRC CanInterfaceMapTest.cpp
	INCLUDES
		${PX4_SOURCE_DIR}/src/drivers/uavcan/uavcan_drivers/stm32h7/driver/include
)
