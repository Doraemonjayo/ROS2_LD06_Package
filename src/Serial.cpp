#include "ld06/Serial.hpp"

Serial::Serial(const char *port_name) : port_name_(port_name), is_open_(false), fd_(-1)
{

}

Serial::~Serial()
{
    end();
}

int Serial::begin(speed_t baudrate){
    baudrate_ = baudrate;

    fd_ = open(port_name_, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ == -1)
    {
        return -1;
    }
    is_open_ = true;

    if (tcgetattr(fd_, &options_) != 0) {
        close(fd_);
        return -1;
    }
    
    cfsetispeed(&options_, baudrate_);
    cfsetospeed(&options_, baudrate_);

    options_.c_cflag |= (CLOCAL | CREAD);
    options_.c_cflag &= ~CSIZE;
    options_.c_cflag |= CS8;
    options_.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    options_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    options_.c_cc[VMIN] = 0;
    options_.c_cc[VTIME] = 0;

    tcflush(fd_, TCIFLUSH);

    if (tcsetattr(fd_, TCSANOW, &options_) != 0) {
        close(fd_);
        return -1;
    }
    return fd_;
}

void Serial::end(){
    if(!is_open_){
        return;
    }
    close(fd_);
    is_open_ = false;
    fd_ = -1;
}

int Serial::available(){
    if(!is_open_){
        return -1;
    }
    int bytesAvailable;
    ioctl(fd_, FIONREAD, &bytesAvailable);
    return bytesAvailable;
}

ssize_t Serial::read(uint8_t *buf, size_t size){
    if(!is_open_){
        return 0;
    }
    return ::read(fd_, buf, size);
}

ssize_t Serial::write(const uint8_t *buf, size_t size){
    if(!is_open_){
        return 0;
    }
    return ::write(fd_, buf, size);
}

bool Serial::isOpen(){
    return is_open_;
}

const char *Serial::getPortName(){
    return port_name_;
}