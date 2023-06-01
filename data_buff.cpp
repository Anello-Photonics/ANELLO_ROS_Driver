#include "data_buff.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>

#ifndef RTCM3PREAMB
#define RTCM3PREAMB 0xD3        /* rtcm ver.3 frame preamble */
#endif

static const unsigned int tbl_CRC24Q[]={
    0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
    0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
    0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
    0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
    0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
    0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
    0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
    0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
    0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
    0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
    0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
    0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
    0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
    0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
    0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
    0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
    0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
    0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
    0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
    0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
    0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
    0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
    0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
    0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
    0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
    0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
    0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
    0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
    0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
    0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
    0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
    0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
};

extern unsigned int crc24q(const unsigned char *buff, int len)
{
    unsigned int crc=0;
    int i;
    
    for (i=0;i<len;i++) crc=((crc<<8)&0xFFFFFF)^tbl_CRC24Q[(crc>>16)^buff[i]];
    return crc;
}

extern void setbitu(unsigned char *buff, int pos, int len, unsigned int data)
{
    unsigned int mask=1u<<(len-1);
    int i;
    if (len<=0||32<len) return;
    for (i=pos;i<pos+len;i++,mask>>=1) {
        if (data&mask) buff[i/8]|=1u<<(7-i%8); else buff[i/8]&=~(1u<<(7-i%8));
    }
}
extern unsigned int getbitu(const unsigned char *buff, int pos, int len)
{
    unsigned int bits=0;
    int i;
    for (i=pos;i<pos+len;i++) bits=(bits<<1)+((buff[i/8]>>(7-i%8))&1u);
    return bits;
}
extern int getbits(const unsigned char *buff, int pos, int len)
{
    unsigned int bits=getbitu(buff,pos,len);
    if (len<=0||32<=len||!(bits&(1u<<(len-1)))) return (int)bits;
    return (int)(bits|(~0u<<len)); /* extend sign */
}

extern int encode_type(unsigned char* buff, int type, int subtype, int sync, unsigned char *msg, int msg_len)
{
    int i=0,nbit=0,len=0,j=0;
    unsigned int crc=0;
    /* set preamble and reserved */
    setbitu(buff,i, 8,RTCM3PREAMB); i+= 8;
    setbitu(buff,i, 6,0          ); i+= 6;
    setbitu(buff,i,10,0          ); i+=10;
    /* body */
    setbitu(buff,i,12,type       ); i+=12; /* message no */
    setbitu(buff,i, 4,subtype    ); i+= 4; /* sub-message no */
    setbitu(buff,i, 1,sync       ); i+= 1; /* multiple message bit */
    setbitu(buff,i,10,msg_len    ); i+=10; /* msg len */
    
    for (j=0;j<msg_len;++j) {
         setbitu(buff,i, 8,msg[j]); i+= 8;
    }
    nbit=i;
    
    /* padding to align 8 bit boundary */
    for (i=nbit;i%8;i++) {
        setbitu(buff,i,1,0);
    }
    /* message length (header+data) (bytes) */
    if ((len=i/8)>=3+1024) {
        /* generate rtcm 3 message length error */
        return 0;
    }
    /* message length without header and parity */
    setbitu(buff,14,10,len-3);
    
    /* crc-24q */
    crc=crc24q(buff,len);
    setbitu(buff,i,24,crc);

    /* length total (bytes) */
    return len+3;
}

extern int decode_data(unsigned char* buff, int nbyte, int *type, int *subtype, int *sync, unsigned char *msg)
{
    int len = 0, i = 24, j = 0, mlen = 0;
    if (nbyte<=3) return 0;
    len=getbitu(buff,14,10)+3; /* length without parity */
    if (nbyte<(len+3)) return 0;

    i = 24;
    *type    = getbitu(buff, i, 12); i += 12;
    *subtype = getbitu(buff, i,  4); i +=  4;
    *sync    = getbitu(buff, i,  1); i +=  1;
     mlen    = getbitu(buff, i, 10); i += 10;

    /* check parity */
    if (crc24q(buff, len) != getbitu(buff, len * 8, 24)) return -1;

    for (j = 0; j < mlen; ++j) {
        msg[j] = getbitu(buff, i, 8); i += 8;
        if (i>(len*8)) return -2;
    }
    return mlen;
}


extern int decode_type(unsigned char* buff, int nbyte)
{
    int len = 0, i = 24, type = 0;
    if (nbyte<=3) return 0;
    len=getbitu(buff,14,10)+3; /* length without parity */
    if (nbyte<(len+3)) return 0;

    i = 24;
    type = getbitu(buff, i, 12); i += 12;
    return type;
}

/*-----------------------------------------------*/
/* ublox decoder */

extern unsigned short U2(unsigned char* p) {
	unsigned short u;
	memcpy(&u, p, 2);
	return u;
}
extern unsigned int U4(unsigned char* p) {
	unsigned int u;
	memcpy(&u, p, 4);
	return u;
}
extern short I2(unsigned char* p) {
	short u;
	memcpy(&u, p, 2);
	return u;
}
extern int I4(unsigned char* p) {
	int u;
	memcpy(&u, p, 4);
	return u;
}
extern float R4(unsigned char* p) {
	float r;
	memcpy(&r, p, 4);
	return r;
}
extern double R8(unsigned char* p) {
	double r;
	memcpy(&r, p, 8);
	return r;
}

extern double I8(unsigned char* p) { return I4(p + 4) * 4294967296.0 + U4(p); }

/* checksum ------------------------------------------------------------------*/
extern int checksum(unsigned char* buff, int len) {
	unsigned char cka = 0, ckb = 0;
	int i;

	for (i = 2; i < len - 2; i++) {
		cka += buff[i];
		ckb += cka;
	}
	return cka == buff[len - 2] && ckb == buff[len - 1];
}

extern int input_ubx_data(ubxbuff_t* ubx, uint8_t data) {
	int ret = 0;
	if (ubx->nbyte >= MAX_BUF_LEN) ubx->nbyte = 0;
	if (ubx->nbyte == 0) memset(ubx, 0, sizeof(ubxbuff_t));
	if (ubx->nbyte == 0 && !(data == UBXSYNC1)) return 0;
	if (ubx->nbyte == 1 && !(data == UBXSYNC2 && ubx->buf[0] == UBXSYNC1)) { ubx->nbyte = 0; return 0; }
	ubx->buf[ubx->nbyte++] = data;
	if (ubx->nbyte >= 6) {
		ubx->nlen = U2(ubx->buf + 4) + 8;
		if (ubx->nbyte >= ubx->nlen) {
			ubx->type = (U1(ubx->buf + 2) << 8) + U1(ubx->buf + 3);
			/* checksum */
			if (!checksum(ubx->buf, ubx->nlen)) {
#ifdef _WIN32
				printf("ubx checksum error: type=%04x len=%d\n", ubx->type, ubx->nlen);
#endif
				ubx->crc = 1;
			} else {
				ubx->crc = 0;
				ubx->type1 = U1(ubx->buf + 2);
				ubx->type2 = U1(ubx->buf + 3);
#ifdef _WIN32
				printf("%x %x\r\n", U1(ubx->buf + 2), U1(ubx->buf + 3));
#endif
				ret = 1;
			}
			ubx->nbyte = 0;
		}
	}
	return ret;
}

#ifdef _OFFLINE_
static FILE* fTMP = NULL;
#endif
extern int decode_ubx_01_07(uint8_t* buff, int len, ubx_nav_pvt_t *nav_pvt) 
{
	unsigned char* p = buff + 6;
	int type1 = 0;
	int type2 = 0;
	int ret = 0;
	int msg_len = 92;
	if (buff[0] == UBXSYNC1 && buff[1] == UBXSYNC2 && len >= (6 + msg_len) && (type1 = U1(buff + 2)) == 0x01 && (type2 = U1(buff + 3)) == 0x07) 
    {
		nav_pvt->iTOW = U4(p + 0); /* U4 ms */
		nav_pvt->year = U2(p + 4); /* U2 */
		nav_pvt->month = U1(p + 6); /* U1 */
		nav_pvt->day = U1(p + 7); /* U1 */
		nav_pvt->hour = U1(p + 8); /* U1 */
		nav_pvt->min = U1(p + 9); /* U1 */
		nav_pvt->sec = U1(p +10); /* U1 */
		nav_pvt->valid = U1(p +11); /* X1 */
		nav_pvt->tAcc = U4(p +12); /* U4 ns */
		nav_pvt->nano = I4(p +16); /* I4 ns */
		nav_pvt->fixType = U1(p +20); /* X1 */
		nav_pvt->flags = U1(p +21); /* X1 */
		nav_pvt->flags2 = U1(p +22); /* X1 */
		nav_pvt->numSV = U1(p +23); /* U1 */
		nav_pvt->lon = I4(p +24); /* I4 1e-7 deg */
		nav_pvt->lat = I4(p +28); /* I4 1e-7 deg  */
		nav_pvt->height = I4(p +32); /* I4 mm */
		nav_pvt->hMSL = I4(p +36); /* I4 mm */
		nav_pvt->hAcc = U4(p +40); /* U4 mm */
		nav_pvt->vAcc = U4(p +44); /* U4 mm */
		nav_pvt->velN = I4(p +48); /* I4 mm/s */
		nav_pvt->velE = I4(p +52); /* I4 mm/s  */
		nav_pvt->velD = I4(p +56); /* I4 mm/s */
		nav_pvt->gSpeed = I4(p +60); /* I4 mm/s */
		nav_pvt->headMot = I4(p +64); /* I4 1.0e-5 deg */
		nav_pvt->sAcc = U4(p +68); /* U4 mm/s */
		nav_pvt->headAcc = U4(p +72); /* U4 1.0e-5 deg */
		nav_pvt->pDOP = U2(p +76); /* U2 0.01 */
		nav_pvt->flags3 = U1(p +78); /* X1 */
		nav_pvt->reserved0[0] = U1(p +79); /* U1 */
		nav_pvt->reserved0[1] = U1(p +80); /* U1 */
		nav_pvt->reserved0[2] = U1(p +81); /* U1 */
		nav_pvt->reserved0[3] = U1(p +82); /* U1 */
		nav_pvt->reserved0[4] = U1(p +83); /* U1 */
		nav_pvt->headVeh = I4(p +84); /* I4 1.0e-5 deg */
		nav_pvt->magDec = I2(p +88); /* I2 1.0e-2 deg */
		nav_pvt->magAcc = U2(p +90); /* U2 1.0e-2 deg */
		ret = 1;
	}
	return ret;
}
extern int decode_ubx_01_14(uint8_t* buff, int len, ubx_nav_hpposllh_t* nav_hpposllh) 
{
	unsigned char* p = buff + 6;
	int type1 = 0;
	int type2 = 0;
	int ret = 0;
	int msg_len = 36;
	if (buff[0] == UBXSYNC1 && buff[1] == UBXSYNC2 && len >= (6 + msg_len) && (type1 = U1(buff + 2)) == 0x01 && (type2 = U1(buff + 3)) == 0x14)
    {
		nav_hpposllh->version = U1(p + 0); /* U1 */
		nav_hpposllh->reserved0 = U2(p + 1); /* U2 */
		nav_hpposllh->flags = U1(p + 3); /* X1 */
		nav_hpposllh->iTOW = U4(p + 4); /* U4 ms */
		nav_hpposllh->lon = I4(p + 8); /* I4 1e-7 deg */
		nav_hpposllh->lat = I4(p +12); /* I4 1e-7 deg */
		nav_hpposllh->height = I4(p +16); /* I4 mm */
		nav_hpposllh->hMSL = I4(p +20); /* I4 mm */
		nav_hpposllh->lonHp = I1(p +24); /* I1 1e-7 deg */
		nav_hpposllh->latHp = I1(p +25); /* I1 1e-7 deg  */
		nav_hpposllh->heightHp = I1(p +26); /* I1 0.1 mm */
		nav_hpposllh->hMSLHp = I1(p +27); /* I1 0.1 mm */
		nav_hpposllh->hAcc = U4(p +28); /* U4 0.1 mm */
		nav_hpposllh->vAcc = U4(p +32); /* U4 0.1 mm */
		ret = 1;
	}
	return ret;
}
extern int decode_ubx_01_3c(uint8_t* buff, int len, ubx_nav_relposned_t* nav_relposned)
{
	unsigned char* p = buff + 6;
	int type1 = 0;
	int type2 = 0;
	int ret = 0;
	int msg_len = 64;
	if (buff[0] == UBXSYNC1 && buff[1] == UBXSYNC2 && len >= (6 + msg_len) && (type1 = U1(buff + 2)) == 0x01 && (type2 = U1(buff + 3)) == 0x3c)
    {
		nav_relposned->version = U1(p);
		nav_relposned->reserved0 = U1(p + 1);
		nav_relposned->refStationId = U2(p + 2);
		nav_relposned->iTOW = U4(p + 4);	 //ms
		nav_relposned->relPosN = I4(p + 8); /* cm */
		nav_relposned->relPosE = I4(p + 12); /* cm */
		nav_relposned->relPosD = I4(p + 16); /* cm */
		nav_relposned->relPosLength = I4(p + 20); /* cm */
		nav_relposned->relPosHeading = I4(p + 24); /* 1e-5 deg */
		nav_relposned->reserved1 = U4(p + 28); /* U1[4] */
		nav_relposned->relPosHPN = I1(p + 32); /* 0.1 mm */
		nav_relposned->relPosHPE = I1(p + 33); /* 0.1 mm */
		nav_relposned->relPosHPD = I1(p + 34); /* 0.1 mm */
		nav_relposned->relPosHPLength = I1(p + 35); /* 0.1 mm */
		nav_relposned->accN = U4(p + 36); /* 0.1 mm */
		nav_relposned->accE = U4(p + 40); /* 0.1 mm */
		nav_relposned->accD = U4(p + 44); /* 0.1 mm */
		nav_relposned->accLength = U4(p + 48); /* 0.1 mm */
		nav_relposned->accHeading = U4(p + 52); /* 1.0e-5 deg */
		nav_relposned->reserved2 = U4(p+56); /* U1[4] */
		nav_relposned->flags = U4(p + 60); /* X4 */
		ret = 1;
	}
	return ret;
}