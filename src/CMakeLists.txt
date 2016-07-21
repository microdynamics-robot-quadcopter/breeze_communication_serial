set(COMMUNICATION_LINK_LIB ../lib/communication_link)

include_directories(
  ../include
  ${COMMUNICATION_LINK_LIB}/include
)

add_executable(
  breeze_communication_serial
  communication_port.cpp
  communication_serial_port.cpp
  communication_serial_interface.cpp
  ${COMMUNICATION_LINK_LIB}/src/communication_link.cpp
  ${COMMUNICATION_LINK_LIB}/src/communication_data_type.cpp
)
target_link_libraries(
  breeze_communication_serial
  ${Boost_LIBRARIES}
)
