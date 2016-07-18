#ifndef COMMUNICATION_SERIAL_PARAM
#define COMMUNICATION_SERIAL_PARAM

#include <sys/types.h>
#include <string>

namespace communication_serial {

class CommunicationSerialParam
{
public:
    CommunicationSerialParam(void) :
        serial_port_("/dev/ttyUSB0"),
        serial_baud_rate_(115200),
        serial_flow_control_(0),
        serial_parity_(0),
        serial_stop_bits_(0)
    {
    }
    CommunicationSerialParam(
        std::string serial_port,
        u_int32_t   serial_baud_rate,
        u_int32_t   serial_flow_control,
        u_int32_t   serial_parity,
        u_int32_t   serial_stop_bits) :
        serial_port_(serial_port),
        serial_baud_rate_(serial_baud_rate),
        serial_flow_control_(serial_flow_control),
        serial_parity_(serial_parity),
        serial_stop_bits_(serial_stop_bits)
    {
    }
public:
    std::string serial_port_;
    u_int32_t   serial_baud_rate_;
    u_int32_t   serial_flow_control_;
    u_int32_t   serial_parity_;
    u_int32_t   serial_stop_bits_;
};

}

#endif // COMMUNICATION_SERIAL_PARAM
