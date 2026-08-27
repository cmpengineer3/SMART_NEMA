#ifndef INC_UID_CONFIG_H_
#define INC_UID_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

#define UID_MAXLEN   24

#define UID_DEFAULT  "KWH012DEMO05" // UID BAWAAN

extern char device_uid[UID_MAXLEN];

/* ── Aksi debug yang ditunda ke superloop (opsi aman) ────────────────────────
 * Perintah debug yang menyentuh MQTT atau publish TIDAK boleh dieksekusi dari
 * dalam wait loop UART (bisa merusak rxBuffer transaksi yang sedang jalan).
 * Jadi UID_Process cuma set flag ini, superloop yang memanggil
 * Debug_HandleDeferred() nanti mengeksekusinya di konteks aman.               */
typedef enum {
    DBG_ACT_NONE = 0,
    DBG_ACT_PUB_FULL,       /* Publish_Full_Payload(&pwr_val, WH)              */
    DBG_ACT_MQTT_CLOSE,     /* MQTT_Close()                                    */
    DBG_ACT_MQTT_DSC,       /* MQTT_Disconnect()                               */
    DBG_ACT_MQTT_OPEN,      /* while(MQTT_Open()!=OK) max 3x, tanpa reset      */
    DBG_ACT_MQTT_CONN,      /* while(MQTT_Connect()!=OK) max 3x, tanpa reset   */
    DBG_ACT_MQTT_SUBS,      /* while(MQTT_Subscribe()!=OK) max 3x, tanpa reset */
} DbgPendingAction;

extern volatile DbgPendingAction dbg_pending;

void UID_Load(void);

bool UID_Save(const char *new_uid);

void UID_FeedByte(uint8_t b);

void UID_Process(void);

void Debug_PrintStatus(void);

/* Wrapper cepat (diimplementasikan di main.c) — dipanggil UID_Process langsung
 * dari konteks apapun karena tidak sentuh UART3/MQTT. */
void Debug_ResetWH_Full(void);      /* reset RAM Wh_R/S/T + timpa FRAM WH=0    */
void Debug_SetRelay(uint8_t on);    /* wrapper Relay_Set                        */

/* Dipanggil dari SUPERLOOP saja — mengeksekusi perintah yang di-set dbg_pending
 * oleh UID_Process. Aman dipanggil kapanpun; kalau dbg_pending == NONE return
 * cepat. */
void Debug_HandleDeferred(void);

#endif /* INC_UID_CONFIG_H_ */
