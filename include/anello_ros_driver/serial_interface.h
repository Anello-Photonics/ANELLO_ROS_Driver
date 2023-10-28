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
// #define DEFAULT_DATA_INTERFACE "/dev/ttyUSB0"
#define DEFAULT_DATA_INTERFACE "AUTO"
#endif

#ifndef DEFAULT_CONFIG_INTERFACE
// #define DEFAULT_CONFIG_INTERFACE "/dev/ttyUSB3"
#define DEFAULT_CONFIG_INTERFACE "AUTO"
#endif

#ifndef PORT_DIR
#define PORT_DIR "/dev/"
#endif

#ifndef PORT_PREFIX
#define PORT_PREFIX "ttyUSB"
#endif

#ifndef MAX_PORT_PARSE_FAIL
#define MAX_PORT_PARSE_FAIL 5
#endif

// #define BAUD_RATE B230400
#define BAUD_RATE B921600

class serial_interface : base_interface
{
protected:
    int usb_fd;
    std::string portname;
    std::vector<std::string> port_names;

    void load_port_names();
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
     * This just sets the variables. To initialize the port, call init().
     */
    serial_interface(const char *ser_port);

    /*
     * Notes:
     * This function initializes the serial port interface including configuring the port.
     * 
     * Return:
     * 0 if successful, 1 if not
     */
    int init();

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
    void free_ports();
};

class anello_config_port : public serial_interface
{

public:
    /*
     * Notes:
     * This constructor does not initialize its port.
     */
    anello_config_port() : serial_interface(){};
    anello_config_port(const char *ser_port_name) : serial_interface(ser_port_name){};

    /*
     * Notes:
     * This function initializes the serial port interface including configuring the port.
     * 
     * Return:
     * 0 if successful, 1 if not
     */
    int init();

};

class anello_data_port : public serial_interface
{
    bool decode_success;
    bool auto_detect;
    uint32_t port_index;
    int fail_count;

public:
    /*
     * Notes:
     * This constructor does not initialize its port.
     */
    anello_data_port() : serial_interface(){};
    anello_data_port(const char *ser_port_name);

    /*
     * Notes:
     * This function initializes the serial port interface including configuring the port.
     * 
     * Return:
     * 0 if successful, 1 if not
     */
    int init();
    size_t get_data(char *buf, size_t buf_len);

    void port_parse_fail();
    void port_confirm();

};     
#endif
