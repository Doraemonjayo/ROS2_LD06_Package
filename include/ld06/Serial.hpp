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
    size_t readBytes(uint8_t *buf, size_t size);
    size_t writeBytes(uint8_t const *buf, size_t size);
    uint8_t read();
    uint8_t write(uint8_t data);
    bool isOpen();
    const char *getPortName();
};

#endif //SERIAL_HPP