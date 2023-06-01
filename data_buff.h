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


int encode_type(unsigned char* buff, int type, int subtype, int sync, unsigned char *msg, int msg_len);
int decode_data(unsigned char* buff, int nbyte, int *type, int *subtype, int *sync, unsigned char *msg);
int decode_type(unsigned char* buff, int nbyte);

#ifndef MAX_BUF_LEN
#define MAX_BUF_LEN (1200)
#endif

typedef struct {
	uint8_t buf[MAX_BUF_LEN];
	int nbyte;
	int nlen; /* length of binary message */
	int type;
	int crc;
	int type1;
	int type2;
} ubxbuff_t;

typedef struct 
{
	uint32_t iTOW; /* U4 ms */
	uint16_t year; /* U2 */
	uint8_t month; /* U1 */
	uint8_t day; /* U1 */
	uint8_t hour; /* U1 */
	uint8_t min; /* U1 */
	uint8_t sec; /* U1 */
	uint8_t valid; /* X1 */
	uint32_t tAcc; /* U4 ns */
	int32_t nano; /* I4 ns */
	uint8_t fixType; /* X1 */
	uint8_t flags; /* X1 */
	uint8_t flags2; /* X1 */
	uint8_t numSV; /* I4 mm */
	int32_t lon; /* I4 1e-7 deg */
	int32_t lat; /* I4 1e-7 deg  */
	int32_t height; /* I4 mm */
	int32_t hMSL; /* I4 mm */
	uint32_t hAcc; /* U4 mm */
	uint32_t vAcc; /* U4 mm */
	int32_t velN; /* I4 mm/s */
	int32_t velE; /* I4 mm/s  */
	int32_t velD; /* I4 mm/s */
	int32_t gSpeed; /* I4 mm/s */
	int32_t headMot; /* I4 1.0e-5 deg */
	uint32_t sAcc; /* U4 mm/s */
	uint32_t headAcc; /* U4 1.0e-5 deg */
	uint16_t pDOP; /* U2 0.01 */
	uint8_t flags3; /* X1 */
	uint8_t reserved0[5]; /* U1 */
	int32_t headVeh; /* I4 1.0e-5 deg */
	int16_t magDec; /* I2 1.0e-2 deg */
	uint16_t magAcc; /* U2 1.0e-2 deg */
} ubx_nav_pvt_t;

typedef struct 
{
	uint8_t version; /* U1 */
	uint16_t reserved0; /* U2 */
	uint8_t flags; /* X1 */
	uint32_t iTOW; /* U4 ms */
	int32_t lon; /* I4 1e-7 deg */
	int32_t lat; /* I4 1e-7 deg */
	int32_t height; /* I4 mm */
	int32_t hMSL; /* I4 mm */
	int8_t lonHp; /* I1 1e-9 deg */
	int8_t latHp; /* I1 1e-9 deg  */
	int8_t heightHp; /* I1 0.1 mm */
	int8_t hMSLHp; /* I1 0.1 mm */
	uint32_t hAcc; /* U4 0.1 mm */
	uint32_t vAcc; /* U4 0.1 mm */
} ubx_nav_hpposllh_t;

typedef struct 
{
	uint8_t version; /* U1*/
	uint8_t reserved0; /* U1 */
	uint16_t refStationId; /* U2 */
	uint32_t iTOW; /* U4 ms */
	int32_t relPosN; /* I4 cm */
	int32_t relPosE; /* I4 cm */
	int32_t relPosD; /* I4 cm */
	int32_t relPosLength; /* I4 cm */
	int32_t relPosHeading; /* I4 cm */
	uint32_t reserved1; /* U4  */
	int8_t relPosHPN; /* I1 0.1 mm */
	int8_t relPosHPE; /* I1 0.1 mm */
	int8_t relPosHPD; /* I1 0.1 mm */
	int8_t relPosHPLength; /* I1 0.1 mm */
	uint32_t accN; /* U4 0.1 mm */
	uint32_t accE; /* U4 0.1 mm */
	uint32_t accD; /* U4 0.1 mm */
	uint32_t accLength; /* U4 0.1 mm */
	uint32_t accHeading; /* U4 1.0e-5 deg */
	uint32_t reserved2; /* U4 ms */
	uint32_t flags; /* U4 ms */
} ubx_nav_relposned_t;

int input_ubx_data(ubxbuff_t* ubx, uint8_t data);

int decode_ubx_01_07(uint8_t* buff, int len, ubx_nav_pvt_t *nav_pvt);
int decode_ubx_01_14(uint8_t* buff, int len, ubx_nav_hpposllh_t* nav_hpposllh);
int decode_ubx_01_3c(uint8_t* buff, int len, ubx_nav_relposned_t* nav_relposned);

#define UBXSYNC1 0xB5 /* ubx message sync code 1 */
#define UBXSYNC2 0x62 /* ubx message sync code 2 */

#define U1(p) (*((unsigned char*)(p)))
#define I1(p) (*((signed char*)(p)))

unsigned short U2(unsigned char* p);
unsigned int U4(unsigned char* p);
short I2(unsigned char* p);
int I4(unsigned char* p);
float R4(unsigned char* p);
double R8(unsigned char* p);
double I8(unsigned char* p);

int checksum(unsigned char* buff, int len);

#define OK_NAV_PVT_ANT1 2 << 0
#define OK_NAV_PVT_ANT2 2 << 1
#define OK_NAV_HPPOSLLH_ANT1 2 << 3
#define OK_NAV_HPPOSLLH_ANT2 2 << 4
#define OK_NAV_RELPOSNED_ANT1 2 << 5
#define OK_NAV_RELPOSNED_ANT2 2 << 6

typedef struct 
{
	/* ubx decoder */
	ubxbuff_t ant1_buff;
	ubxbuff_t ant2_buff;

	/* nav pvt */
	ubx_nav_pvt_t nav_pvt_ant1;
	ubx_nav_pvt_t nav_pvt_ant2;

	/* nav hpposllh */
	ubx_nav_hpposllh_t nav_hpposllh_ant1;
	ubx_nav_hpposllh_t nav_hpposllh_ant2;

	/* nav relposned, the antenna have this message will be the secondard antenna */
	ubx_nav_relposned_t nav_relposned;

	/* GNSS data status */
	uint8_t status;

} gnss_t;

#ifdef __cplusplus
}
#endif

#endif
