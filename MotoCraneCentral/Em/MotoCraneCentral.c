/**** DO NOT EDIT -- this file has been automatically generated from @emmoco.com.MotoCraneCentral on 2015-09-08T23:34:46 ****/
/**** protocolLevel = 14, toolsVersion = 14.3.3.201409231615 ****/

#include "Em_Message.h"
#include "MotoCraneCentral.h"

#ifdef __cplusplus
extern "C" {
#endif

#define Em_Message_protocolLevel 14

typedef struct Em_App_Message {
    uint8_t dummy[3];
    uint8_t sot;
    Em_Message_Header hdr;
    uint8_t data[20];
} Em_App_Message;

const uint8_t Em_App_hash[] = {196, 191, 23, 50, 213, 230, 1, 38, 96, 46, 116, 166, 93, 86, 22, 41, 14, 0, ((sizeof(struct{uint8_t f1; uint16_t f2;}) - sizeof(uint16_t)) << 4) | (sizeof(struct{uint8_t f1; uint32_t f2;}) - sizeof(uint32_t))};

const uint8_t Em_App_build[] = {105, 80, 208, 176, 79, 1, 0, 0};

#define Em_App_NUM_IMPORTS 0
const uint8_t Em_App_imports[] = {0};

#define Em_App_APP_RESOURCE_COUNT 3
#define Em_App_SYS_RESOURCE_COUNT 9

#define Em_App_ACCEPT MotoCraneCentral_accept
#define Em_App_ACTIVATEPARAMETERS MotoCraneCentral_activateParameters
#define Em_App_BEACON MotoCraneCentral_beacon
#define Em_App_BROADCASTOFF MotoCraneCentral_broadcastOff
#define Em_App_DISCONNECT MotoCraneCentral_disconnect
#define Em_App_ONCOMMANDDONE MotoCraneCentral_onCommandDone
#define Em_App_PAIRINGON MotoCraneCentral_pairingOn
#define Em_App_PAIRINGOFF MotoCraneCentral_pairingOff
#define Em_App_RESET MotoCraneCentral_reset
#define Em_App_SCAN MotoCraneCentral_scan
#define Em_App_SETBEACONINFO MotoCraneCentral_setBeaconInfo
#define Em_App_SETDEVICENAME MotoCraneCentral_setDeviceName
#define Em_App_START MotoCraneCentral_start

#define Em_App_CONNECTHANDLER MotoCraneCentral_connectHandler
#define Em_App_DISCONNECTHANDLER MotoCraneCentral_disconnectHandler
#define Em_App_SCANDONEHANDLER MotoCraneCentral_scanDoneHandler

#define Em_App_NUM_CHANNELS 3

/* BEGIN common code */

#include <string.h>

enum {Em_App_IDLE, Em_App_STARTING, Em_App_DISCONNECTED, Em_App_CONNECTED};
enum {Em_App_FETCH_NONE, Em_App_FETCH_PENDING, Em_App_FETCH_ACTIVE};

typedef struct Em_App_Indicator {
    uint8_t dummy[3];
    uint8_t sot;
    Em_Message_Header hdr;
    uint8_t data[Em_Message_INDSIZE];
} Em_App_Indicator;

union { uint32_t align; Em_App_Message msg; } Em_App_msg_u;
union { uint32_t align; Em_App_Indicator ind; } Em_App_ind_u;
union { uint32_t align; Em_App_Indicator cmd; } Em_App_cmd_u;
#define Em_App_msg Em_App_msg_u.msg
#define Em_App_ind Em_App_ind_u.ind
#define Em_App_cmd Em_App_cmd_u.cmd

void (*Em_App_pdHdlr)(void);

const uint16_t Em_App_endian = 0x0100;

Em_Message_Size Em_App_recvIdx;
Em_Message_Size Em_App_recvSize;
Em_Message_Size Em_App_xmitIdx;
Em_Message_Size Em_App_xmitSize;

uint8_t Em_App_state = Em_App_IDLE;
Em_FileDataFxn Em_App_fileDataFxn;
Em_DoneFxn Em_App_fileDoneFxn;
int32_t Em_App_fileIndex = 0;
Em_Message_ResId Em_App_fileResId;
uint32_t Em_App_xmitMask = 0;

uint8_t* Em_App_valp;
uint8_t* Em_App_bufp;
const char* Em_App_desc;

uint8_t* Em_App_dataPtr = (uint8_t*)&Em_App_msg.data;
uint8_t* Em_App_inBuf = (uint8_t*)&Em_App_msg.hdr;
uint8_t* Em_App_outBuf = 0;

uint8_t Em_App_fetchState = Em_App_FETCH_NONE;

uint8_t* _Em_Message_rxBuf = 0;
uint8_t _Em_Message_rxCnt = 0;

uint8_t* _Em_Message_txBuf = 0;
uint8_t _Em_Message_txCnt = 0;

#define Em_App_DEVNAME_LEN 9
const char* Em_App_devName = 0;

#define Em_App_UUID_LEN 16
struct {
    const uint8_t* uuid;
    uint16_t major;
    uint16_t minor;
    int8_t power;
} Em_App_beaconInfo = {0, 0, 0, 0};

void (*Em_App_curCb)() = 0;
void (*Em_App_curCmdDoneCb)() = 0;
void (*Em_App_curIndDisp)() = 0;

void Em_App_connect(Em_Device* device, uint8_t timeout, Em_DoneFxn done, Em_DoneFxn disp);
void Em_App_fetchDispatch(void);
void Em_App_file(bool readFlag, Em_Message_ResId resId, Em_FileDataFxn dataFxn, Em_DoneFxn doneFxn);
void Em_Message_marshallToBuf(uint8_t* valp, uint8_t* bufp, const char* desc);
void Em_Message_marshallToVal(uint8_t* valp, uint8_t* bufp, const char* desc);
void Em_App_read(Em_Message_ResId resId, Em_DoneFxn done);
void Em_App_readNext(void);
void Em_App_scanDispatch(Em_Device* dev);
void Em_App_sendCmd(Em_Message_Kind kind, Em_Message_Size size, Em_Message_ResId resId);
void Em_App_sendIndicator(Em_Message_ResId indId);
void Em_App_sendPacket(Em_Message_Kind kind, Em_Message_Size size);
void Em_App_startSend(uint8_t* buffer, uint8_t size);
void Em_App_startCmdSend(void);
void Em_App_startResSend(void);
void Em_App_storeDispatch(void);
void Em_App_sysFetchDispatch(void);
void Em_App_sysStoreDispatch(void);
void Em_App_write(Em_Message_ResId resId, Em_Message_Size size, Em_DoneFxn done);
void Em_App_writeNext(void);
bool Em_App_xmitReady(Em_Message_ResId indId);

void Em_Message_nextXmit(void) {
    uint8_t key = Em_Hal_lock();
    if (Em_App_xmitMask != 0) {
        uint8_t i;
        uint32_t m;
        for (i = 0, m = 0x1; i < Em_App_NUM_CHANNELS; i++, m <<= 1) {
            if (Em_App_xmitMask & m) {
                Em_App_xmitMask &= ~m;
                if (i == 0) {
                    Em_App_startResSend();
                }
                else if (i == 1) {
                    Em_App_startCmdSend();
                }
                else {
                    Em_App_sendIndicator(i - 2);
                }
                break;
            }
        }
    }
    if (Em_App_fetchState != Em_App_FETCH_NONE) {
        Em_App_inBuf = (uint8_t*)&Em_App_msg.hdr;
        Em_App_fetchState = Em_App_FETCH_ACTIVE;
    }
    Em_Hal_unlock(key);
}

void Em_Message_restart(void) {
    Em_App_START();
}

void Em_App_ACCEPT(bool enable) {
    if (Em_App_state == Em_App_CONNECTED) {
        return;
    }
    Em_App_sendCmd(Em_Message_ACCEPT, sizeof (Em_Message_Header), enable);
}

void Em_App_ACTIVATEPARAMETERS(uint8_t group) {
    if (Em_App_state == Em_App_IDLE || Em_App_state == Em_App_STARTING) {
        return;
    }
    Em_App_sendCmd(Em_Message_ACTIVE_PARAMS, sizeof (Em_Message_Header), group);
}

void Em_App_BEACON(uint8_t mode) {
    Em_App_sendCmd(Em_Message_BEACON, sizeof (Em_Message_Header), mode);
}

void Em_App_BROADCASTOFF(void) {
    Em_App_sendCmd(Em_Message_INDICATOR, sizeof (Em_Message_Header), 0);
}

void Em_App_connect(Em_Device* device, uint8_t timeout, Em_DoneFxn done, Em_DoneFxn disp) {
    uint8_t i;
    for (i = 0; i < Em_ADDRLEN; i++) {
        Em_App_cmd.data[i] = device->addr[i];
    }
    Em_App_curCb = done;
    Em_App_curIndDisp = disp;
    Em_App_inBuf = (uint8_t*)&Em_App_ind.hdr;
    Em_App_sendCmd(Em_Message_CONNECT, sizeof (Em_Message_Header) + Em_ADDRLEN, timeout);
}

void Em_App_DISCONNECT(void) {
    if (Em_App_state != Em_App_CONNECTED) {
        return;
    }
    Em_App_state = Em_App_DISCONNECTED;
    Em_App_sendCmd(Em_Message_DISCONNECT, sizeof (Em_Message_Header), 0);
}

void Em_Message_dispatch(void) {
    Em_Device* dev;
    void (*cb)();
    int8_t i;
    if (Em_App_state == Em_App_IDLE) {
        return;
    }
    Em_Message_Kind kind = ((Em_Message_Header*)Em_App_inBuf)->kind;
    switch (kind) {
        case Em_Message_CONNECT:
            Em_App_state = Em_App_CONNECTED;
            if (Em_App_curCb) {
                cb = Em_App_curCb;
                Em_App_curCb = 0;
                cb();
            }
            else {
                Em_App_CONNECTHANDLER();
            }
            break;
        case Em_Message_DISCONNECT:
            Em_App_curCb = 0;
            Em_App_curIndDisp = 0;
            Em_App_state = Em_App_DISCONNECTED;
            Em_App_inBuf = (uint8_t*)&Em_App_msg.hdr;
            Em_App_DISCONNECTHANDLER();
            break;
#ifdef Em_App_OBSERVER
        case Em_Message_SCAN_DONE:
            for (i = 0, dev = (Em_Device*) Em_App_msg.data; i < Em_App_msg.hdr.resId; i++, dev++) {
                Em_App_scanDispatch(dev);
            }
            Em_App_SCANDONEHANDLER((uint8_t)Em_App_msg.hdr.resId);
            break;
        case Em_Message_INDICATOR:
            if (Em_App_fetchState == Em_App_FETCH_ACTIVE) {
                memcpy(&Em_App_msg.hdr, &Em_App_ind.hdr, Em_App_msg.hdr.size);
            }
            if (Em_App_curIndDisp) {
                Em_App_curIndDisp();
            }
            break;
        case Em_Message_FETCH_DONE:
            Em_App_fetchState = Em_App_FETCH_NONE;
            Em_App_inBuf = (uint8_t*)&Em_App_ind.hdr;
            /* fall through */
        case Em_Message_STORE_DONE:
            if (Em_App_curCb) {
                cb = Em_App_curCb;
                Em_App_curCb = 0;
                cb();
            }
            break;
#endif            
        case Em_Message_PAIRING_DONE:
            if (Em_App_pdHdlr) {
                (*Em_App_pdHdlr)();
            }
            break;
        case Em_Message_FETCH:
            if (Em_App_msg.hdr.resId < 0x80) {
                Em_App_fetchDispatch();
            }
            else {
                Em_App_sysFetchDispatch();
            }
            break;
        case Em_Message_STORE:
            if (Em_App_msg.hdr.resId < 0x80) {
                Em_App_storeDispatch();
            }
            else {
                Em_App_sysStoreDispatch();
            }
            break;
        case Em_Message_CMD_DONE:
            if (Em_App_curCmdDoneCb) {
                Em_App_curCmdDoneCb();
            }
            break;
    }
}

void Em_App_file(bool readFlag, Em_Message_ResId resId, Em_FileDataFxn dataFxn, Em_DoneFxn doneFxn) {
    Em_App_fileResId = resId;
    Em_App_fileDataFxn = dataFxn;
    Em_App_fileDoneFxn = doneFxn;
    Em_App_fileIndex = -1;
    Em_App_msg.data[0] = Em_App_msg.data[1] = 0;
    Em_App_write(Em_Message_SYS_FILE_INDEX_RESET, 2, (readFlag ? Em_App_readNext : Em_App_writeNext)); 
}

void Em_App_marshallToBuf() {
    char ch;
    while ((ch = *Em_App_desc++)) {
        switch (ch) {
            case '0' : {
                *Em_App_bufp++ = 0;
                break;
            }
            case '1' : {
                *Em_App_bufp++ = *Em_App_valp & 0xFF;
                break;
            }
            case '2' : {
                uint16_t v16 = *(uint16_t*)Em_App_valp;
                *Em_App_bufp++ = v16 & 0xFF;
                *Em_App_bufp++ = (v16 >> 8) & 0xFF;
                break;
            }
            case '4' : {
                if (((uint32_t)Em_App_valp & 0x1)) Em_App_valp++;
                uint32_t v32 = *(uint32_t*)Em_App_valp++;
                *Em_App_bufp++ = v32 & 0xFF;
                *Em_App_bufp++ = (v32 >> 8) & 0xFF;
                *Em_App_bufp++ = (v32 >> 16) & 0xFF;
                *Em_App_bufp++ = (v32 >> 24) & 0xFF;
                break;
            }
        }
        Em_App_valp += 1;
    }
}

void Em_App_marshallToVal() {
    char ch;
    while ((ch = *Em_App_desc++)) {
        switch (ch) {
            case '0' : {
                *Em_App_valp = 0;
                Em_App_bufp += 1;
                break;
            }
            case '1' : {
                *Em_App_valp = *Em_App_bufp++ & 0xFF;
                break;
            }
            case '2' : {
                uint16_t v16 = *Em_App_bufp++ & 0xFF;
                v16 |= (*Em_App_bufp++ << 8);
                *(uint16_t*)Em_App_valp = v16;
                break;
            }
            case '4' : {
                if (((uint32_t)Em_App_valp & 0x1)) Em_App_valp++;
                uint32_t v32 = (uint32_t)*Em_App_bufp++ & 0xFF;
                v32 |= ((uint32_t)*Em_App_bufp++ << 8);
                v32 |= ((uint32_t)*Em_App_bufp++ << 16);
                v32 |= ((uint32_t)*Em_App_bufp++ << 24);
                *(uint32_t*)Em_App_valp++ = v32;
                break;
            }
        }
        Em_App_valp += 1;
    }
}

void Em_App_ONCOMMANDDONE(Em_DoneFxn callback) {
   Em_App_curCmdDoneCb = callback; 
}

void Em_App_PAIRINGOFF(void(*handler)(void)) {
    Em_App_PAIRINGON(0, handler);
}

void Em_App_PAIRINGON(uint8_t secs, void(*handler)(void)) {
    if (Em_App_state != Em_App_DISCONNECTED) {
        return;
    }
    Em_App_pdHdlr = handler;
    Em_App_sendCmd(Em_Message_PAIRING, sizeof (Em_Message_Header), secs);
}

void Em_App_read(Em_Message_ResId resId, Em_DoneFxn done) {
    Em_App_msg.sot = 0;
    Em_App_msg.hdr.kind = Em_Message_FETCH;
    Em_App_msg.hdr.size = sizeof (Em_Message_Header);
    Em_App_msg.hdr.resId = resId;
    Em_App_curCb = done;
    Em_App_fetchState = Em_App_FETCH_PENDING;
    Em_App_startResSend();
}

void Em_App_readNext(void) {
    if (Em_App_fileIndex < 0) {
        Em_App_fileIndex = 0;
    }
    else {
        uint8_t sz = Em_App_msg.hdr.size - sizeof (Em_Message_Header);
        Em_App_fileDataFxn(&Em_App_msg.data[0], Em_App_fileIndex, sz);
        if (sz < Em_Message_DATASIZE) {
            if (Em_App_fileDoneFxn) {
                Em_App_fileDoneFxn();
            }
            return;
        }
        Em_App_fileIndex += Em_Message_DATASIZE;
    }
    Em_App_msg.hdr.chan = 1;
    Em_App_read(Em_App_fileResId, Em_App_readNext);  
}

void Em_App_RESET(void) {
    Em_Hal_reset();
    _Em_Message_rxBuf = _Em_Message_txBuf = 0;
    _Em_Message_rxCnt = _Em_Message_txCnt = 0;
    Em_App_recvIdx = Em_App_recvSize = Em_App_xmitIdx = Em_App_xmitSize = 0;
    Em_App_state = Em_App_IDLE;
    Em_App_fileIndex = 0;
    Em_App_xmitMask = 0;
}

void Em_App_SETBEACONINFO(const uint8_t* uuid, uint16_t major, uint16_t minor, int8_t power) {
    Em_App_beaconInfo.uuid = uuid;
    Em_App_beaconInfo.major = major;
    Em_App_beaconInfo.minor = minor;
    Em_App_beaconInfo.power = power;
}

void Em_App_SETDEVICENAME(const char* name) {
    Em_App_devName = name;
}

void Em_App_SCAN(uint16_t msecs, uint32_t mask) {
    Em_Message_ScanArgs* args;
    if (Em_App_state == Em_App_DISCONNECTED || Em_App_state == Em_App_CONNECTED) {
        args = (Em_Message_ScanArgs*)Em_App_cmd.data;
        args->msecs = msecs;
        args->mask = mask;
        Em_App_sendCmd(Em_Message_SCAN, sizeof (Em_Message_Header) + sizeof (Em_Message_ScanArgs), 0);
    }
}

void Em_App_START(void) {
    Em_App_RESET();
    Em_App_state = Em_App_STARTING;
}

void Em_App_sendCmd(Em_Message_Kind kind, Em_Message_Size size, Em_Message_ResId resId) {
    Em_App_Indicator* msg = kind == Em_Message_INDICATOR ? &Em_App_ind : &Em_App_cmd;
    msg->sot = 0;
    msg->hdr.kind = kind;
    msg->hdr.size = size;
    msg->hdr.resId = resId;
    if (kind == Em_Message_INDICATOR) {
        Em_App_outBuf = (uint8_t*)&Em_App_ind.sot;
        Em_App_xmitSize = Em_App_ind.hdr.size + 1;
        Em_App_xmitIdx = 0;
        Em_Hal_startSend();
    }
    else if (Em_App_xmitReady(1)) {
        Em_App_startCmdSend();
    }
}

void Em_App_sendPacket(Em_Message_Kind kind, Em_Message_Size size) {
    if (Em_App_state != Em_App_IDLE) {
        Em_App_msg.sot = 0;
        Em_App_msg.hdr.kind = kind;
        Em_App_msg.hdr.size = size + sizeof (Em_Message_Header);
        if (Em_App_xmitReady(0)) {
            Em_App_startResSend();
        }
    }
}

void Em_App_startCmdSend(void) {
    Em_App_startSend((uint8_t*)&Em_App_cmd.sot, Em_App_cmd.hdr.size);
}

void Em_App_startResSend(void) {
    Em_App_startSend((uint8_t*)&Em_App_msg.sot, Em_App_msg.hdr.size);
}

void Em_App_startSend(uint8_t* buf, uint8_t size) {
    Em_App_outBuf = buf;
    Em_App_xmitSize = size + 1;
    Em_App_xmitIdx = 0;
    Em_Hal_startSend();
}

void Em_App_sysFetchDispatch(void) {
    uint8_t size = 0;
    int i, j;
    switch (Em_App_msg.hdr.resId) {
        case Em_Message_SYS_MCM_BEACON:
            if (Em_App_beaconInfo.uuid) {
                for (i = 0; i < Em_App_UUID_LEN; i++) {
                    Em_App_msg.data[i] = Em_App_beaconInfo.uuid[i];
                }
                Em_App_msg.data[i++] = Em_App_beaconInfo.major & 0xFF;                
                Em_App_msg.data[i++] = (Em_App_beaconInfo.major >> 8) & 0xFF;                
                Em_App_msg.data[i++] = Em_App_beaconInfo.minor & 0xFF;                
                Em_App_msg.data[i++] = (Em_App_beaconInfo.minor >> 8) & 0xFF;                
                Em_App_msg.data[i++] = Em_App_beaconInfo.power;
                size = i;
            }            
            break;
        case Em_Message_SYS_MCM_NAME:
            if (Em_App_devName) {
                for (i = 0; i < Em_App_DEVNAME_LEN; i++) {
                    if ((Em_App_msg.data[i] = Em_App_devName[i]) == 0) {
                        break;
                    }
                }
                for (j = i; j < Em_App_DEVNAME_LEN; j++) {
                    Em_App_msg.data[j] = 0;
                }
                size = Em_App_DEVNAME_LEN;
            }
            break;
        case Em_Message_SYS_SCHEMA_HASH:
            for (i = 0; i < sizeof (Em_App_hash); i++) {
                Em_App_msg.data[i] = Em_App_hash[i];
            }
            Em_App_msg.data[sizeof (Em_App_hash)] = *((uint8_t*)&Em_App_endian);
            size = sizeof (Em_App_hash) + 1;
            break;
        case Em_Message_SYS_EAP_BUILD_DATE:
            for (i = 0; i < sizeof (Em_App_build); i++) {
                Em_App_msg.data[i] = Em_App_build[i];
            }
            size = sizeof (Em_App_build);
            break;
        case Em_Message_SYS_EAP_IMPORTS:
            if (Em_App_NUM_IMPORTS) {
                for (i = 0; i < sizeof (Em_App_imports); i++) {
                    Em_App_msg.data[i] = Em_App_imports[i];
                }
            size = sizeof (Em_App_imports);
            }
            break;
        case Em_Message_SYS_EAP_PROTOCOL_LEVEL:
            *((Em_Message_protocolLevel_t*)Em_App_msg.data) = Em_Message_protocolLevel;
            size = sizeof (Em_Message_protocolLevel_t);
            break;
        case Em_Message_SYS_RESOURCE_COUNT:
            Em_App_msg.data[0] = Em_App_APP_RESOURCE_COUNT;
            Em_App_msg.data[1] = Em_App_SYS_RESOURCE_COUNT;
            size = 2;
            break;
    }
    Em_App_sendPacket(Em_Message_FETCH_DONE, size);
}

void Em_App_sysStoreDispatch(void) {
    switch (Em_App_msg.hdr.resId) {
        case Em_Message_SYS_FILE_INDEX_RESET:
            Em_App_fileIndex = 0;
            break;
    }
    Em_App_sendPacket(Em_Message_STORE_DONE, 0);
}

void Em_App_write(Em_Message_ResId resId, Em_Message_Size size, void (*done)()) {
    Em_App_msg.sot = 0;
    Em_App_msg.hdr.kind = Em_Message_STORE;
    Em_App_msg.hdr.size = size + sizeof (Em_Message_Header);
    Em_App_msg.hdr.resId = resId;
    Em_App_curCb = done;
    Em_App_startResSend();
}

void Em_App_writeNext(void) {
    if (Em_App_fileIndex < 0) {
        Em_App_fileIndex = 0;
    }
    uint8_t sz = Em_App_fileDataFxn(&Em_App_msg.data[0], Em_App_fileIndex, Em_Message_DATASIZE);
    Em_App_write(Em_App_fileResId, sz, sz < Em_Message_DATASIZE ? Em_App_fileDoneFxn : Em_App_writeNext);  
    Em_App_fileIndex += Em_Message_DATASIZE;
}

bool Em_App_xmitReady(Em_Message_ResId indId) {
    uint8_t key = Em_Hal_lock();
    bool res = _Em_Message_txBuf == 0 && Em_App_xmitMask == 0;
    if (!res) {
        Em_App_xmitMask |= (1 << indId);
    }
    Em_Hal_unlock(key);
    return res;    
}

/* END common code */

void Em_App_fetchDispatch(void) {
    uint8_t size = 0;
    switch (Em_App_msg.hdr.resId) {
        case 0: {
            break;
        }
        case 2: {
#ifdef Em_16BIT
            MotoCraneCentral_status_t val;
            Em_App_valp = (uint8_t*)&val;
            Em_App_bufp = Em_App_msg.data;
            Em_App_desc = "*\x0b[1]";
            MotoCraneCentral_status_fetch(&val);
            Em_App_marshallToBuf();
#else
            MotoCraneCentral_status_fetch((void*)Em_App_msg.data);
#endif
            size = 11;
            break;
        }
        case 3: {
#ifdef Em_16BIT
            MotoCraneCentral_track_t val;
            Em_App_valp = (uint8_t*)&val;
            Em_App_bufp = Em_App_msg.data;
            Em_App_desc = "*\x05[2]";
            MotoCraneCentral_track_fetch(&val);
            Em_App_marshallToBuf();
#else
            MotoCraneCentral_track_fetch((void*)Em_App_msg.data);
#endif
            size = 10;
            break;
        }
    }
    Em_App_sendPacket(Em_Message_FETCH_DONE, size);
}

void Em_App_storeDispatch(void) {
    switch (Em_App_msg.hdr.resId) {
        case 0: {
            break;
        }
        case 1: {
#ifdef Em_16BIT
            MotoCraneCentral_packet_t val;
            Em_App_valp = (uint8_t*)&val;
            Em_App_bufp = Em_App_msg.data;
            Em_App_desc = "*\x0f[1]";
            Em_App_marshallToVal();
            MotoCraneCentral_packet_store(&val);
#else
            MotoCraneCentral_packet_store((void*)Em_App_msg.data);
#endif
            break;
        }
    }
    Em_App_sendPacket(Em_Message_STORE_DONE, 0);
}

void Em_App_sendIndicator(Em_Message_ResId indId) {
}

#ifdef __cplusplus
}
#endif

