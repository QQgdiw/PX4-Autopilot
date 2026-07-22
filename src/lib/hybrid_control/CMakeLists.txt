px4_add_library(hybrid_control
	C610Protocol.cpp
	M2006CommandAdapter.cpp
	M2006DriveGate.cpp
	M2006SpeedController.cpp
	TransformationPosition.cpp
	TransformationStateMachine.cpp
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
