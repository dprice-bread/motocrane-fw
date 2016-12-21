#ifndef Em_Types_H_
#define Em_Types_H_

#ifndef Em_NOSTDBOOL
#include <stdbool.h>
#endif

#ifndef Em_NOSTDINT
#include <stdint.h>
#endif

#ifdef Em_16BIT
typedef signed char     int8_t;
typedef unsigned char   uint8_t;
#endif

#ifndef NULL
#define NULL ((void*)0)
#endif

typedef void (*Em_DoneFxn)(void);
typedef uint16_t (*Em_FileDataFxn)(uint8_t* iobuf, int32_t offset, uint16_t count);

#define Em_ADDRLEN 6
#define Em_NAMELEN 20
#define Em_DATALEN 4

#define Em_BCAST_DATA_MASK 0x80

typedef struct Em_Device {
    uint8_t     __schId;
    int8_t      rssi;
    uint8_t     addr[Em_ADDRLEN];
    char        name[Em_NAMELEN];
    uint8_t    __data[Em_DATALEN];
} Em_Device; 

#endif /*Em_Types_H_*/
