include_directories(../include)

add_executable(
  breeze_communication_serial
  communication_port.cpp
  communication_serial_port.cpp
)
target_link_libraries(
  breeze_communication_serial
  ${Boost_LIBRARIES}
)
