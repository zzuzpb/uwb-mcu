#ifndef __CRC_H__
#define __CRC_H__

// CRC
extern const unsigned char crc8_tab[];
#define crc8_init(crc) ((crc) = 0XACU)
#define crc8(crc, v) ( (crc) = crc8_tab[(crc) ^(v)])

// calc crc8 for for 8, 16, 32 integer
#define CRC8_8(crc , v) crc8(crc, v)
#define CRC8_16(crc, v) { unsigned char v0 = (v)&0xFFU, v1 = ((v)>>8)&0xFFU; crc8(crc, v0), crc8(crc,v1); }
#define CRC8_32(crc ,v) { unsigned char v0 = (v)&0xFFU, v1 = ((v)>>8)&0xFFU, v2 = ((v)>>16)&0xFFU, v3 = ((v)>>24)&0xFFU; crc8(crc, v0), crc8(crc,v1), crc8(crc, v2), crc8(crc, v3); }

#endif
