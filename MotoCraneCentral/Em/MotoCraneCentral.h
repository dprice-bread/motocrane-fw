/**** DO NOT EDIT -- this file has been automatically generated from @emmoco.com.MotoCraneCentral on 2015-09-08T23:34:46 ****/
/**** protocolLevel = 14, toolsVersion = 14.3.3.201409231615 ****/

#ifndef MotoCraneCentral__H
#define MotoCraneCentral__H

#include "Em_Types.h"
#include "Em_Message.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------- resource types defined in the schema -------- */

/* resource packet */
typedef int8_t MotoCraneCentral_packet_t[15];
#define MotoCraneCentral_packet_length 15

/* resource status */
typedef int8_t MotoCraneCentral_status_t[11];
#define MotoCraneCentral_status_length 11

/* resource track */
typedef int16_t MotoCraneCentral_track_t[5];
#define MotoCraneCentral_track_length 5

/* -------- resource callbacks implemented by the application -------- */

/* resource packet */
extern void MotoCraneCentral_packet_store(MotoCraneCentral_packet_t input);

/* resource status */
extern void MotoCraneCentral_status_fetch(MotoCraneCentral_status_t output);

/* resource track */
extern void MotoCraneCentral_track_fetch(MotoCraneCentral_track_t output);

/* -------- peripheral connection functions implemented in MotoCraneCentral.c -------- */

void MotoCraneCentral_accept(bool enable);
void MotoCraneCentral_activateParameters(uint8_t group);
void MotoCraneCentral_beacon(uint8_t mode);
void MotoCraneCentral_broadcastOff(void);
void MotoCraneCentral_disconnect(void);
void MotoCraneCentral_onCommandDone(Em_DoneFxn doneFxn);
void MotoCraneCentral_pairingOn(uint8_t secs, void(*handler)(void));
void MotoCraneCentral_pairingOff(void(*handler)(void));
void MotoCraneCentral_reset(void);
void MotoCraneCentral_setBeaconInfo(const uint8_t[], uint16_t major, uint16_t minor, int8_t power);
void MotoCraneCentral_setDeviceName(const char* name);
void MotoCraneCentral_start(void);

/* -------- peripheral connection callbacks implemented by the application -------- */

void MotoCraneCentral_connectHandler(void);
void MotoCraneCentral_disconnectHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* MotoCraneCentral__H */
