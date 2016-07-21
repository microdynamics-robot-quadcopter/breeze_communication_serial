#ifndef COMMUNICATION_SERIAL_INTERFACE_H
#define COMMUNICATION_SERIAL_INTERFACE_H

#define COMMUNICATION_SERIAL_INTERFACE_LIB 0

#include <fstream>
#include <communication_link.h>
#include <communication_serial_port.h>

namespace communication_serial {

typedef boost::shared_ptr<CommunicationSerialPort>     CommSerialPort;
typedef boost::shared_ptr<CommunicationLink>           CommLink;
typedef boost::shared_ptr<boost::asio::deadline_timer> Timer;

class CommunicationSerialInterface
{
public:
    CommunicationSerialInterface(std::string serial_url,
                                 std::string config_addr);
    void checkShakeHandState(void);
    bool getFlagInit(void);
    bool updateCommandState(const CommunicationCommandState &command_state,
                            int count);
    IO getIOInstace(void);
    CommunicationDataType *getDataType(void);
private:
    void runTimeoutHandler(const boost::system::error_code &error_code);
    void sendCommand(const CommunicationCommandState command_state);
    u_int8_t checkUpdateState(const CommunicationCommandState command_state);
private:
    int                   timeout_;
    int                   link_command_set_[LAST_COMMAND];
    int                   link_command_set_current_[LAST_COMMAND];
    int                   link_command_frequency_[LAST_COMMAND];
    int                   link_command_count_[LAST_COMMAND];
    bool                  flag_timeout_;
    bool                  flag_init_;
    bool                  flag_ack_;
    std::fstream          config_file_;
    boost::mutex          mutex_wait_;
    CommSerialPort        serial_port_;
    CommLink              serial_link_;
    Timer                 timer_;
    CommunicationDataType data_type_;
};

}

#endif // COMMUNICATION_SERIAL_INTERFACE_H
