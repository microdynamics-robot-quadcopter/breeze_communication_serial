#include <iostream>
#include <communication_serial_port.h>

namespace communication_serial {

CommunicationSerialPort::CommunicationSerialPort(void) :
    CommunicationPort("serial:///dev/ttyUSB0")
{
    serial_param_.serial_port_ = "/dev/ttyUSB0";

    if (!initializeSerialPort()) {
        std::cerr << "Serial port initialize unsuccessfully!" << std::endl;
        flag_init_ = false;
    }
    else {
        std::cout << "Serial port initialize succesfully!" << std::endl;
        flag_init_ = true;
    }
}

CommunicationSerialPort::CommunicationSerialPort(std::string serial_url) :
    CommunicationPort(serial_url)
{
    if (comm_url_.substr(0, comm_url_.find("://")) != "serial") {
        std::cerr << "URL is error!" << std::endl;
        return ;
    }

    serial_param_.serial_port_ = comm_url_.substr(comm_url_.find("://") + 3,
                                                  comm_url_.length() -
                                                  comm_url_.find("://"));

    if (!initializeSerialPort()) {
        std::cerr << "Serial port initialize unsuccessfully!" << std::endl;
        flag_init_ = false;
    }
    else {
        std::cout << "Serial port initialize succesfully!" << std::endl;
        flag_init_ = true;
    }
}

Buffer CommunicationSerialPort::readBuffer(void)
{
    boost::mutex::scoped_lock lock(mutex_read_);
    Buffer data;

    if (!buffer_read_.empty()) {
        Buffer data(buffer_read_.front());
        buffer_read_.pop();
        return data;
    }

    return data;
}

void CommunicationSerialPort::writeBuffer(Buffer &buffer)
{
    boost::mutex::scoped_lock lock(mutex_write_);

    buffer_write_.push(buffer);
    startWrite();
}

void CommunicationSerialPort::startRead(void)
{
    boost::mutex::scoped_lock lock(mutex_port_);

    port_->async_read_some(boost::asio::buffer(buffer_temp_),
                           boost::bind(&CommunicationSerialPort::runReadHandler,
                                       this,
                                       boost::asio::placeholders::error,
                                       boost::asio::placeholders::bytes_transferred));
}

void CommunicationSerialPort::startWrite(void)
{
    boost::mutex::scoped_lock lock(mutex_port_);

    if (!buffer_write_.empty()) {
        boost::asio::async_write(*port_, boost::asio::buffer(buffer_write_.front()),
                                 boost::bind(&CommunicationSerialPort::runWriteHandler,
                                             this, boost::asio::placeholders::error));
        buffer_write_.pop();
    }
}

void CommunicationSerialPort::runMainThread(void)
{
    std::cout << "Start serial port read/write thread!" << std::endl;
    startRead();
    io_service_->run();
}

void CommunicationSerialPort::runReadHandler(
    const boost::system::error_code &error_code,
    u_int32_t trans_bytes)
{
    if (error_code) {
        std::cerr << "Read serial port error!" << std::endl;
        return ;
    }

    boost::mutex::scoped_lock lock(mutex_read_);
    Buffer data(buffer_temp_.begin(), buffer_temp_.begin() + trans_bytes);
    buffer_read_.push(data);
    startRead();
}

void CommunicationSerialPort::runWriteHandler(
    const boost::system::error_code &error_code)
{
    if (error_code) {
        std::cerr << "Write serial port error!" << std::endl;
        return ;
    }

    boost::mutex::scoped_lock lock(mutex_write_);

    if (!buffer_write_.empty()) {
        startWrite();
    }
}

bool CommunicationSerialPort::initializeSerialPort(void)
{
    try {
        port_ = boost::make_shared<boost::asio::serial_port>(
                    boost::ref(*io_service_),
                    serial_param_.serial_port_);
        port_->set_option(boost::asio::serial_port::baud_rate(
                   serial_param_.serial_baud_rate_));
        port_->set_option(boost::asio::serial_port::flow_control(
                   (boost::asio::serial_port::flow_control::type)serial_param_.serial_flow_control_));
        port_->set_option(boost::asio::serial_port::parity(
                   (boost::asio::serial_port::parity::type)serial_param_.serial_parity_));
        port_->set_option(boost::asio::serial_port::stop_bits(
                   (boost::asio::serial_port::stop_bits::type)serial_param_.serial_stop_bits_));
        port_->set_option(boost::asio::serial_port::character_size(8));
    }
    catch(std::exception &exce) {
        std::cerr << "Open the serial port unsuccessfully!" << std::endl;
        std::cerr << "Error information: " << exce.what() << std::endl;
        return false;
    }

    buffer_temp_.resize(1024);

    try {
        thread_ = boost::thread(boost::bind(
                                    &CommunicationSerialPort::runMainThread,
                                    this));
    }
    catch (std::exception &exce) {
        std::cerr << "Create the serial thread unsuccessfully!" << std::endl;
        std::cerr << "Error information: " << exce.what() << std::endl;
        return false;
    }

    return true;
}

}

int main(void)
{
    return 0;
}
