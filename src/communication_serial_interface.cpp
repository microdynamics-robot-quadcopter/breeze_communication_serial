#include <communication_serial_interface.h>

namespace communication_serial {

CommunicationSerialInterface::CommunicationSerialInterface(
    std::string serial_url,
    std::string config_addr)
{
    std::string serial_port_mode = serial_url.substr(0, serial_url.find("://"));

    if (serial_port_mode == "serial") {
        serial_port_ = boost::make_shared<CommunicationSerialPort>(serial_url);
        timeout_ = 500;
        serial_link_ = boost::make_shared<CommunicationLink>(0x01, 0x11,
                                                             &data_type_);
        timer_.reset(new boost::asio::deadline_timer(
                             *(serial_port_->getIOInstance()),
                             boost::posix_time::milliseconds(timeout_)));
    }
    else {
    }

    config_file_.open(config_addr.c_str(), std::fstream::in);

    if (config_file_.is_open()) {
        for (int i = 0; i < LAST_COMMAND; i++) {
            std::string command_name;
            config_file_ >> command_name >> link_command_set_[i]
                         >> link_command_frequency_[i];
            std::cout << command_name << link_command_set_[i]
                      << link_command_frequency_[i] << std::endl;
        }
        config_file_.close();
        flag_init_ = serial_port_->getFlagInit();
    }
    else {
        std::cerr << "Config file can't be opened!" << std::endl;
        flag_init_ = false;
    }
}

void CommunicationSerialInterface::checkShakeHandState(void)
{
    if (serial_link_->getReceiveState(SHAKE_HANDS)) {
        sendCommand(SHAKE_HANDS);
        std::cout << "Send shake hands command" << std::endl;
    }
}

bool CommunicationSerialInterface::getFlagInit(void)
{
    return flag_init_;
}

bool CommunicationSerialInterface::updateCommandState(
    const CommunicationCommandState &command_state,
    int count)
{
    boost::asio::deadline_timer circle_timer_(*(serial_port_->getIOInstance()));
    circle_timer_.expires_from_now(boost::posix_time::milliseconds(timeout_));

    if (!link_command_set_[command_state]) {
        int cnt = count % 100;
        if (cnt % (100 / link_command_frequency_[command_state]) == 0) {
            sendCommand(command_state);
        }
        else {
            return false;
        }
    }

    Buffer data = serial_port_->readBuffer();
    flag_ack_ = false;

    while (!flag_ack_) {
        for (int i = 0; i < data.size(); i++) {
            if (serial_link_->analyseReceiveByte(data[i])) {
                flag_ack_ = true;
            }
        }
        data = serial_port_->readBuffer();
        if (circle_timer_.expires_from_now().is_negative()) {
            std::cerr << "Timeout, skip this package!" << std::endl;
            return false;
        }
    }

    return true;
}

IO CommunicationSerialInterface::getIOInstace(void)
{
    return serial_port_->getIOInstance();
}

CommunicationDataType *CommunicationSerialInterface::getDataType(void)
{
    return &data_type_;
}

void CommunicationSerialInterface::runTimeoutHandler(
    const boost::system::error_code &error_code)
{
    if (!error_code) {
        std::cerr << "Timeout!" << std::endl;
        boost::mutex::scoped_lock lock(mutex_wait_);
        flag_timeout_ = true;
    }
}

void CommunicationSerialInterface::sendCommand(
    const CommunicationCommandState command_state)
{
    std::cout << "Send command: " << command_state << std::endl;
    serial_link_->sendCommandFromMaster(command_state);
    Buffer data(serial_link_->getSerializeData(),
                serial_link_->getSerializeData() +
                serial_link_->getSerializedLength());
    serial_port_->writeBuffer(data);
}

u_int8_t CommunicationSerialInterface::checkUpdateState(
    const CommunicationCommandState command_state)
{
    if (link_command_set_current_[command_state] &
        serial_link_->getReceiveState(command_state)) {
        return TRUE;
    }
    if (!link_command_set_current_[command_state]) {
        return TRUE;
    }

    return FALSE;
}

}

#if !COMMUNICATION_SERIAL_INTERFACE_LIB
int main(void)
{
    return 0;
}
#endif
