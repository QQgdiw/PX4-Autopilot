px4_add_library(hx8_protocol Hx8Protocol.cpp)
target_include_directories(hx8_protocol PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

px4_add_library(hx8_controller Hx8Controller.cpp)
target_include_directories(hx8_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hx8_controller PUBLIC hx8_protocol)

px4_add_unit_gtest(SRC Hx8ProtocolTest.cpp LINKLIBS hx8_protocol)
px4_add_unit_gtest(SRC Hx8ControllerTest.cpp LINKLIBS hx8_controller)
px4_add_unit_gtest(SRC Hx8CliPolicyTest.cpp)

px4_add_library(hx65_protocol Hx65Protocol.cpp)
target_include_directories(hx65_protocol PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

px4_add_unit_gtest(SRC Hx65ProtocolTest.cpp LINKLIBS hx65_protocol)

px4_add_library(hx65_pair_controller Hx65PairController.cpp)
target_include_directories(hx65_pair_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hx65_pair_controller PUBLIC hx65_protocol)

px4_add_unit_gtest(SRC Hx65PairControllerTest.cpp LINKLIBS hx65_pair_controller)
