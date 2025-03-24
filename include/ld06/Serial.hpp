#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>

class Serial
{
private:
    int fd_;
    struct termios options_;
    const char* const port_name_;
    speed_t baudrate_;
    bool is_open_;
public:
    Serial(const char *port_name);
    ~Serial();
    int begin(speed_t baudrate);
    void end();
    int available();
    ssize_t read(uint8_t *buf, size_t size);
    ssize_t write(uint8_t const *buf, size_t size);
    bool isOpen();
    const char *getPortName();
};

#endif //SERIAL_HPP