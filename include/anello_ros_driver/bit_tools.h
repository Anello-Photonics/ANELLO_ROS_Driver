#ifndef __DATA_BUFF_H__
#define __DATA_BUFF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

unsigned int crc24q(const unsigned char* buff, int len);

void setbitu(unsigned char* buff, int pos, int len, unsigned int data);
unsigned int getbitu(const unsigned char* buff, int pos, int len);
int getbits(const unsigned char *buff, int pos, int len);

/*
* Parameters:
* unsigned char* buff : buffer containing full ASCII message
* int len : length of the message
*
* Return:
* int
* non-zero : checksum at the end of the message matches the calculated checksum
* zero : The checksum is not correct and the message is invalid
*
*/
int checksum(unsigned char* buff, int len); 

#ifndef MAX_BUF_LEN
#define MAX_BUF_LEN (1200)
#endif

#ifdef __cplusplus
}
#endif

#endif
