/********************************************************************************
 * File Name:   anello_data_port.h
 * Description: header file for the anello_data_port class.
 *
 * Author:      Austin Johnson
 * Date:        7/12/24
 *
 * License:     MIT License
 *
 * Note:        The anello_data_port class is used to read/write data to the anello GNSS INS, EVK, and IMU+.
 *              This class is meant to abstract the UART/ethernet communication with the anello devices.
 *
 ********************************************************************************/

#ifndef ANELLO_DATA_PORT_H
#define ANELLO_DATA_PORT_H

#include "serial_interface.h"
#include "ethernet_interface.h"

class anello_data_port
{
private:
    bool decode_success;
    bool auto_detect;
    std::vector<std::string> port_names;
    uint32_t port_index;
    int fail_count;
    
    interface_config_t config;
    serial_interface uart_port;
    ethernet_interface ethernet_port;

    void init_ethernet();
    void init_uart();

    void port_parse_fail_uart();
    void port_parse_fail_ethernet();
    void port_confirm_uart();
    void port_confirm_ethernet();

    size_t get_data_uart(char *buf, size_t buf_len);
    size_t get_data_ethernet(char *buf, size_t buf_len);

    void write_data_uart(const char *buf, size_t buf_len);
    void write_data_ethernet(const char *buf, size_t buf_len);
public:
    /*
     * Notes:
     * This constructor does not initialize its port.
     */
    anello_data_port(const interface_config_t *config);
    ~anello_data_port();

    /*
     * Notes:
     * This function initializes the serial port interface including configuring the port.
     */
    void init();
    size_t get_data(char *buf, size_t buf_len);
    void write_data(const char *buf, size_t buf_len);

    void port_parse_fail();
    void port_confirm();

};     

#endif // ANELLO_CONFIG_PORT_H