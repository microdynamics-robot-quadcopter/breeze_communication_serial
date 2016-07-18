#ifndef COMMUNICATION_PORT_H
#define COMMUNICATION_PORT_H

#include <queue>
#include <string>
#include <vector>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>

namespace communication_serial {

typedef std::vector<u_int8_t> Buffer;
typedef boost::shared_ptr<boost::asio::io_service> IO;

class CommunicationPort
{
public:
    CommunicationPort(std::string comm_url);
    virtual Buffer readBuffer(void) = 0;
    virtual void writeBuffer(Buffer &buffer) = 0;
    bool getFlagInit(void);
    IO getIOInstance(void);
protected:
    bool               flag_init_;
    std::string        comm_url_;
    std::queue<Buffer> buffer_read_;
    std::queue<Buffer> buffer_write_;
    IO                 io_service_;
};

}

#endif // COMMUNICATION_PORT_H
