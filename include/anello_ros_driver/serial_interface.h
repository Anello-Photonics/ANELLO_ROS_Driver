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

#include <vector>
#include <string>

#include "base_interface.h"

#ifndef DEFAULT_DATA_INTERFACE
#define DEFAULT_DATA_INTERFACE "/dev/ttyUSB0"
#endif

#ifndef DEFAULT_CONFIG_INTERFACE
#define DEFAULT_CONFIG_INTERFACE "/dev/ttyUSB3"
#endif

#ifndef PORT_DIR
#define PORT_DIR "/dev/"
#endif

#ifndef PORT_PREFIX
#define PORT_PREFIX "ttyUSB"
#endif

#ifndef BAUDRATE
#define BAUDRATE B230400    //Default baudrate for ANELLO GNSS INS and IMU+
// #define BAUDRATE B921600    //Default baudrate for ANELLO EVK
#endif

#ifndef MAX_PORT_PARSE_FAIL
#define MAX_PORT_PARSE_FAIL 5
#endif

class serial_interface : base_interface
{
protected:
    bool port_enabled;
    int usb_fd;
    std::string portname;

public:

    /*
     * Parameters:
     * const char *ser_port : string defining the serial port that the object will listen too
     *
     * Notes:
     * This just sets the variables. To initialize the port, call init().
     */
    serial_interface();

    /*
     * Notes:
     * This function initializes the serial port interface including configuring the port.
     */
    void init(std::string portname);

    /*
     * Parameters:
     * char *buf : char array that will be written over with as much ANELLO unit data as possible
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
     * int timeout : time in milliseconds to wait for data
     *
     * Return:
     * Number of bytes written to the port
     */
    size_t get_data(char *buf, size_t buf_len, int timeout);

    /*
     * Parameters:
     * char *buf : char array that will be written to the connected device
     * size_t buf_len : amount of bytes to be written to the port
     *
     */
    void write_data(const char *buf, size_t buf_len);
    
    /*
     * Parameters:
     *
     * Return:
     * string containing the port name
     */
    const std::string get_portname() const;

    /*
     * Parameters:
     *
     * Return:
     * bool value of the port status
     */
    bool get_port_enabled();

    /*
     * Notes:
     * Closes the port serial port
     */
    void close_port();

    /*
     * Notes:
     * Closes the port serial port
     */
    ~serial_interface();
};
#endif