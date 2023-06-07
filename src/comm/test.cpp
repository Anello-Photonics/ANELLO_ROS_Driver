/** @file serial_interface.c
*
*
* @brief This is an file that defines the object that will represent the serial connection with the EVK
*
*
*
*/


#include <fcntl.h>
#include <cstring>
#include <termios.h>
#include <unistd.h>
#include <string>

#include "../../include/anello_ros_driver/serial_interface.h"

#define EVK_SERIAL_DATA_PORT "/dev/ttyUSB0"
#define EVK_SERIAL_CONFIG_PORT "/dev/ttyUSB3"
#define DEBUG 0
#define MAX_READ_NUM 100
#define MAX_BUF_LEN 128

// static int setup_serial(const char *portname);
// static int read_serial_data(int fd, char *buff, size_t buf_len);

int main (int argc, char *argv[]){
    uint32_t read_count = 0;
    int usb_fd;
    int bytes_read;
    char ser_buff[MAX_BUF_LEN];

    std::string str_config(EVK_SERIAL_DATA_PORT);
    const char *data_port_str = str_config.c_str();

    serial_interface data_port(data_port_str);

    while (MAX_READ_NUM > read_count)
    {
        bytes_read = data_port.get_data(ser_buff, MAX_BUF_LEN);
        if (bytes_read > 0)
        {
            printf("%s", ser_buff);
            read_count++;
        }

    }
    close(usb_fd);    
}

serial_interface::serial_interface ()
{
    this->usb_fd = -1;
}

// static int setup_serial (const char *portname)
serial_interface::serial_interface (const char *portname)
{
    this->usb_fd = open(portname, O_RDWR);   
    if (this->usb_fd < 0){
        printf("file open error");
        exit(1);
    }

    struct termios options; 
    memset(&options, 0, sizeof(options));
    if (tcgetattr(this->usb_fd, &options) != 0) {
        printf("Error %i from tcgetattr: %s\n",errno, strerror(errno));
        exit(1);
    }

    options.c_cflag &= ~PARENB;         // disable parity bit
    options.c_cflag &= ~CSTOPB;         // use one stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;             // 8-bit data
    options.c_cflag &= ~CRTSCTS;        // disable hardware flow control
    options.c_lflag = 0;                // no signaling characters, no echo
    options.c_oflag = 0;                // no remapping, no delays
    options.c_cc[VMIN] = 0;             // read doesn't block
    options.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    cfsetispeed(&options, B921600);
    cfsetospeed(&options, B921600);

    if (tcsetattr(this->usb_fd, TCSANOW, &options) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        exit(1);
    }
}

size_t serial_interface::get_data (char *buf, size_t buf_len)
{
    if (this->usb_fd < 0)
    {
        printf("Anell ros driver serial port file escriptor not defined");
    }
    printf("Test 1\n");
    size_t bytes_read = read(usb_fd, buf, buf_len);
    printf("Test 2\n");
    buf[bytes_read] = '\0';
    return bytes_read;
}