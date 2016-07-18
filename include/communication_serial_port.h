#ifndef COMMUNICATION_SERIAL_PORT_H
#define COMMUNICATION_SERIAL_PORT_H

#include "communication_port.h"
#include "communication_serial_param.h"

namespace communication_serial {

typedef boost::shared_ptr<boost::asio::serial_port> Port;

class CommunicationSerialPort : public CommunicationPort
{
public:
    CommunicationSerialPort(void);
    CommunicationSerialPort(std::string serial_url);
    Buffer readBuffer(void);
    void writeBuffer(Buffer &buffer);
private:
    void startRead(void);
    void startWrite(void);
    void runMainThread(void);
    void runReadHandler(const boost::system::error_code &error_code,
                        u_int32_t trans_bytes);
    void runWriteHandler(const boost::system::error_code &error_code);
    bool initializeSerialPort(void);
private:
    Buffer                   buffer_temp_;
    boost::thread            thread_;
    boost::mutex             mutex_port_;
    boost::mutex             mutex_read_;
    boost::mutex             mutex_write_;
    CommunicationSerialParam serial_param_;
    Port                     port_;
};

}

#endif // COMMUNICATION_SERIAL_PORT_H
