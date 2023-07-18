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
    /*
    * Notes:
    * This constructor does not initialize its port and should not be used.
    */
    serial_interface();
    
    /*
    * Parameters:
    * const char *ser_port : string defining the serial port that the object will listen too
    *
    * Notes:
    * This constructor initializes the serial port interface including configuring the port.
    */
    serial_interface(const char *ser_port);

    /*
    * Parameters:
    * char *buf : char array that will be written over with as much anello unit data as possible 
    * size_t buf_len : size of the buffer in 'char *buf'
    *
    * Return:
    * Number of bytes written 'char *buf'
    */
    size_t get_data(char *buf, size_t buf_len);

    /*
    * Notes:
    * Closes the port serial port
    */
    ~serial_interface();
};

#endif
