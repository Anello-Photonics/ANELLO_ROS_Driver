/********************************************************************************
 * File Name:   anello_config_port.h
 * Description: header file for the anello_config_port class.
 *
 * Author:      Austin Johnson
 * Date:        7/12/24
 *
 * License:     MIT License
 *
 * Note:        The anello_config_port class is used to read/write data to the ANELLO GNSS INS, EVK, and IMU+.
 *              This class is meant to abstract the UART/ethernet communication with the ANELLO devices.
 *
 ********************************************************************************/

#ifndef ANELLO_CONFIG_PORT_H
#define ANELLO_CONFIG_PORT_H

#include "serial_interface.h"
#include "ethernet_interface.h"

class anello_config_port
{
public:
    /*
     * Notes:
     * This constructor does not initialize its port.
     */
    anello_config_port(const interface_config_t *config);

    ~anello_config_port();

    /*
     * Notes:
     * This function initializes the serial port interface including configuring the port.
     */
    void init();

    size_t get_data(char *buf, size_t buf_len);
    void write_data(const char *buf, size_t buf_len);

    double get_baseline();

    const std::string get_portname() const {
        if (this->config.type == ETH) {
            return this->ethernet_port.get_remote_ip(); // Assuming get_remote_ip() exists
        } else {
            return this->uart_port.get_portname();
        }
    }

private:
    interface_config_t config;
    serial_interface uart_port;
    ethernet_interface ethernet_port;

    void init_ethernet();
    void init_uart();

    size_t get_data_uart(char *buf, size_t buf_len);
    size_t get_data_ethernet(char *buf, size_t buf_len);

    void write_data_uart(const char *buf, size_t buf_len);
    void write_data_ethernet(const char *buf, size_t buf_len);
};

#endif // ANELLO_CONFIG_PORT_H