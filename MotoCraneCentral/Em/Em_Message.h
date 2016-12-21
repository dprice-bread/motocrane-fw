#ifndef Em_Message_H_
#define Em_Message_H_

#include "Em_Types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------- SRT FUNCTIONS CALLED BY HAL -------- */

static inline bool Em_Message_addByte(uint8_t b);
extern void Em_Message_dispatch(void);
static inline bool Em_Message_getByte(uint8_t* bp);
extern void Em_Message_restart(void);
static inline bool Em_Message_startRx(void);
static inline uint8_t Em_Message_startTx(void);


/* -------- HAL FUNCTIONS CALLED BY SRT -------- */

extern uint8_t Em_Hal_lock(void);
extern void Em_Hal_reset(void);
extern void Em_Hal_startSend(void);
extern void Em_Hal_unlock(uint8_t key);
extern void Em_Hal_watchOff(void);
extern void Em_Hal_watchOn(void);


/* -------- MESSAGE FORMAT -------- */

#define Em_Message_DATASIZE 240

/* protocolLevel #4 */
/* protocolLevel #14 -- increase size from 4 for DEVICE messages */
#define Em_Message_INDSIZE 32

typedef uint8_t Em_Message_Size;
typedef uint8_t Em_Message_Kind;
/* protocolLevel #12 -- split 16-bit resId into <resId, chan> */
typedef uint8_t Em_Message_ResId;
typedef uint8_t Em_Message_Chan;

#define Em_Message_NOP 0
#define Em_Message_FETCH 1
#define Em_Message_FETCH_DONE 2
#define Em_Message_STORE 3
#define Em_Message_STORE_DONE 4
#define Em_Message_INDICATOR 5
#define Em_Message_CONNECT 6
#define Em_Message_DISCONNECT 7
#define Em_Message_ECHO 8
/* protocolLevel #3 */
/* protocolLevel #6  -- rename from BROADCAST to PAIRING */
#define Em_Message_PAIRING 9
#define Em_Message_PAIRING_DONE 10
/* protocolLevel #7 */
#define Em_Message_OFFLINE 11
/* protocolLevel #8 */
#define Em_Message_ACCEPT 12
/* protocolLevel #13 */
#define Em_Message_START 13
#define Em_Message_ACTIVE_PARAMS 14
/* protocolLevel #14 */
#define Em_Message_SCAN 15
#define Em_Message_SCAN_DONE 16
#define Em_Message_BEACON 17
#define Em_Message_CMD_DONE 18

typedef struct Em_Message_ScanArgs {
    uint32_t mask;
    uint16_t msecs;
} Em_Message_ScanArgs;

typedef struct Em_Message_Header {
    Em_Message_Size size;
    Em_Message_Kind kind;
    Em_Message_ResId resId;
    Em_Message_Chan chan;
} Em_Message_Header;

typedef uint16_t Em_Message_protocolLevel_t;

/* protocolLevel #1 */

/* protocolLevel #10 */
/* #define Em_Message_SYS_SCHEMA_UUID       0xFF */

#define Em_Message_SYS_MCM_PROTOCOL_LEVEL   0xFE
#define Em_Message_SYS_EAP_PROTOCOL_LEVEL   0xFD
#define Em_Message_SYS_EAP_BUILD_DATE       0xFC

/* protocolLevel #2 */
#define Em_Message_SYS_FILE_INDEX_RESET     0xFB

/* protocolLevel #5 */
#define Em_Message_SYS_SCHEMA_HASH          0xFA

/* protocolLevel #7 */
#define Em_Message_SYS_RESOURCE_COUNT       0xF9

/* protocolLevel #9 */
#define Em_Message_SYS_MOBILE_RSSI          0xF8

/* protocolLevel #11 */
#define Em_Message_SYS_MCM_DISCONNECT       0xF7

/* protocolLevel #13a */
#define Em_Message_SYS_MCM_NAME             0xF5

/* protocolLevel #14 */
#define Em_Message_SYS_EAP_IMPORTS          0xF4

/* protocolLevel #14 */
#define Em_Message_SYS_MCM_BEACON           0xF3


/* -------- PRIVATE -------- */

extern void Em_Message_nextXmit(void);

extern uint8_t* Em_App_dataPtr;
extern uint8_t* Em_App_inBuf;
extern uint8_t* Em_App_outBuf;
extern Em_Message_Size Em_App_xmitSize;

extern uint8_t* _Em_Message_rxBuf;
extern uint8_t _Em_Message_rxCnt;

extern uint8_t* _Em_Message_txBuf;
extern uint8_t _Em_Message_txCnt;

static inline bool Em_Message_addByte(uint8_t b) {
    if (_Em_Message_rxCnt == 0) {
        if (b == 0) {
            return false;
        }
        _Em_Message_rxCnt = b;
    }
    *_Em_Message_rxBuf++ = b;
    if (--_Em_Message_rxCnt == 0) {
        _Em_Message_rxBuf = 0;
        if (_Em_Message_txBuf == 0) {
            Em_Hal_watchOff();
        }
        return true;
    }
    else {
        return false;
    }
}

static inline bool Em_Message_getByte(uint8_t* bp) {
    if (_Em_Message_txBuf == 0) {
        return false;
    }
    if (_Em_Message_txCnt == 0) {
        _Em_Message_txCnt = *_Em_Message_txBuf + 1;
    }
    if (--_Em_Message_txCnt > 0) {
        *bp = *_Em_Message_txBuf++;
        return true;
    }
    else {
        _Em_Message_txBuf = 0;
        Em_App_xmitSize = 0;
        Em_Message_nextXmit();
        if (_Em_Message_rxBuf == 0) {
            Em_Hal_watchOff();
        }
        return false;
    }
}

static inline bool Em_Message_startRx(void) {
    if (_Em_Message_rxBuf == 0) {
        _Em_Message_rxBuf = Em_App_inBuf;
        if (_Em_Message_txBuf == 0) {
            Em_Hal_watchOn();
        }
        return true;
    }
    else {
        return false;
    }
}

static inline uint8_t Em_Message_startTx(void) {
    _Em_Message_txBuf = Em_App_outBuf + 1;
    _Em_Message_txCnt = 0;
    if (_Em_Message_rxBuf == 0) {
        Em_Hal_watchOn();
    }
    return 0;
}


#ifdef __cplusplus
}
#endif

#endif /*Em_Message_H_*/
