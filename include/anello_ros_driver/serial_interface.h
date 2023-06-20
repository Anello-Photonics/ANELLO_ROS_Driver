#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include "base_interface.h"

#ifndef DEFAULT_DATA
#define default_serial_interface "/dev/ttyUSB0"
#endif

#ifndef default_config_interface
#define default_config_interface "/dev/ttyUSB3"
#endif

class serial_interface : base_interface{
    int usb_fd;
public:
    serial_interface();
    serial_interface(const char *ser_port);
    size_t get_data(char *buf, size_t buf_len);
    ~serial_interface();
};

#endif
