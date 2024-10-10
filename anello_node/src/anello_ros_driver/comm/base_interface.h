/********************************************************************************
 * File Name:   base_interface.h
 * Description: Contains the base implementation used for all ANELLO device interfaces.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 *
 * Note:        All interfaces to the ANELLO devices should inherit from this class.
 ********************************************************************************/

#ifndef BASE_INTERFACE_H
#define BASE_INTERFACE_H

// enum with options UART and ETH
enum interface_type_t
{
	UART,
	ETH
};

typedef struct
{
	interface_type_t type;
	std::string data_port_name;
	std::string config_port_name;

	std::string remote_ip;
	int local_data_port;
	int local_config_port;
	int local_odometer_port;
} interface_config_t;

class base_interface
{
protected:
    interface_config_t config;

public:
    /*
     * Parameters:
     * char *buf : char array that will be written over with as much anello unit data as possible
     * size_t buf_len : size of the buffer in 'char *buf'
     *
     * Return:
     * Number of bytes written 'char *buf'
     */
    virtual size_t get_data(char *buf, size_t buf_len) { (void)buf; (void)buf_len; return 0; }
    
    /*
     * Parameters:
     * char *buf : char array that will be written to the connected device
     * size_t buf_len : amount of bytes to be written to the port
     *
     */
    virtual void write_data(const char *buf, size_t buf_len) { (void)buf; (void)buf_len; return; }
};

#endif
