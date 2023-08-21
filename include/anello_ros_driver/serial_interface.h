/********************************************************************************
 * File Name:   serial_interface.h
 * Description: header file for the serial_interface class.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 *
 * Note:        This class is used to read data from a serial port.
 *              To use this class, create an object with the serial port as a parameter.
 *              Then repeatedly call get_data() to read data from the serial port.
 *
 *              This class inherits from base_interface.
 *
 *              This default serial port is /dev/ttyUSB0. This can be changed with the constructor parameter.
 ********************************************************************************/

#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include "base_interface.h"

#ifndef DEFAULT_DATA_INTERFACE
#define DEFAULT_DATA_INTERFACE "/dev/ttyUSB0"
#endif

#ifndef DEFAULT_CONFIG_INTERFACE
#define DEFAULT_CONFIG_INTERFACE "/dev/ttyUSB3"
#endif

class serial_interface : base_interface
{
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
     * Parameters:
     * char *buf : char array that will be written to the connected device
     * size_t buf_len : amount of bytes to be written to the port
     *
     */
    void write_data(const char *buf, size_t buf_len);

    /*
     * Notes:
     * Closes the port serial port
     */
    ~serial_interface();
};

#endif
