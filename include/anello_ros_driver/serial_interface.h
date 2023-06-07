#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include "base_interface.h"

class serial_interface {
    int usb_fd;
public:
    serial_interface();
    serial_interface(const char *ser_port);
    size_t get_data(char *buf, size_t buf_len);
    ~serial_interface();
};

#endif
