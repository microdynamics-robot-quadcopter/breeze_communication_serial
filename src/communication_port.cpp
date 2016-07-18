#include <communication_port.h>

namespace communication_serial {

CommunicationPort::CommunicationPort(std::string comm_url) :
    comm_url_(comm_url)
{
    io_service_ = IO();
}

bool CommunicationPort::getFlagInit(void)
{
    return flag_init_;
}

IO CommunicationPort::getIOInstance(void)
{
    return io_service_;
}

}
