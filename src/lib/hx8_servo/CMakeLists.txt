px4_add_library(hx8_protocol Hx8Protocol.cpp)
target_include_directories(hx8_protocol PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

px4_add_library(hx8_controller Hx8Controller.cpp)
target_include_directories(hx8_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hx8_controller PUBLIC hx8_protocol)

px4_add_unit_gtest(SRC Hx8ProtocolTest.cpp LINKLIBS hx8_protocol)
px4_add_unit_gtest(SRC Hx8ControllerTest.cpp LINKLIBS hx8_controller)
