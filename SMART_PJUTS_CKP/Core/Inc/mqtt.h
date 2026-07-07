/*
 * mqtt.h — Quectel EC25 MQTT driver (modular)
 *
 * Usage:
 *   1. Populate an MQTT_Config struct with broker credentials.
 *   2. Call MQTT_Init(&cfg) once before MQTT_Open().
 *   3. Call MQTT_Open → MQTT_Connect → MQTT_Subscribe / MQTT_Publish.
 *   4. In the main loop, call MQTT_ProcessIncoming() to handle received data.
 *
 * Dependency: uart.h (which only needs stm32f4xx_hal.h).
 * Copy mqtt.h + mqtt.c + uart.h + uart.c to any project.
 */

#ifndef INC_MQTT_H_
#define INC_MQTT_H_

#include "uart.h"
#include <stddef.h>

/* ── Status type ──────────────────────────────────────────────────────────── */
typedef enum {
    MQTT_OK    = 0,
    MQTT_ERROR = 1
} MQTT_StatusTypeDef;

/* ── Broker / credential configuration ───────────────────────────────────── */
typedef struct {
    char broker_host[64];
    int  broker_port;
    char client_id[64];
    char username[64];  /* leave empty string "" for no-auth brokers */
    char password[64];
} MQTT_Config;

/* ── Init ─────────────────────────────────────────────────────────────────── */
/*
 * Store broker credentials. Call once before MQTT_Open().
 * Subsequent calls update the credentials (for runtime broker switching).
 */
void MQTT_Init(const MQTT_Config *cfg);

/* ── Utility ──────────────────────────────────────────────────────────────── */
uint16_t Modbus_CRC16(const unsigned char *buf, size_t len);

/*
 * Ekstrak nilai object "DATA" dari payload terima.
 * Payload contoh: {"CRC":"08B8","DATA":{"H":"R"}}
 * Hasil di out:   {"H":"R"}
 * Mengembalikan true jika berhasil parsing.
 */
bool MQTT_ExtractDataObject(const char *payload, char *out, size_t out_size);

/*
 * Validasi CRC pada payload masuk.
 * Format payload yang divalidasi: {"CRC":"XXXX","DATA":{...}}
 * CRC dihitung dari string isi object DATA (termasuk braces {…}).
 *
 * Return:
 *   true  → CRC ada dan cocok
 *   false → CRC tidak ada / format salah / CRC tidak cocok
 */
bool MQTT_VerifyPayloadCRC(const char *payload);

/* ── Network & session ────────────────────────────────────────────────────── */
MQTT_StatusTypeDef MQTT_CheckNetwork(void);
MQTT_StatusTypeDef MQTT_Open(void);
MQTT_StatusTypeDef MQTT_Connect(void);
MQTT_StatusTypeDef MQTT_Disconnect(void);
MQTT_StatusTypeDef MQTT_Close(void);

/*
 * Disconnect → Close → Open → Connect.
 * After this, call MQTT_Subscribe() again if needed.
 */
MQTT_StatusTypeDef MQTT_Reconnect(void);

/* ── Publish / Subscribe ──────────────────────────────────────────────────── */
MQTT_StatusTypeDef MQTT_Subscribe(const char *topic, int qos);
MQTT_StatusTypeDef MQTT_Publish(const char *topic, const char *payload,
                                int qos, int retain);

/*
 * Publish dengan auto-CRC.
 * Argumen data_object: hanya isi object DATA, contoh: "{\"H\":\"K\",\"VR\":221.2,...}"
 * Fungsi akan otomatis membungkus menjadi:
 *   {"CRC":"XXXX","DATA":<data_object>}
 * dengan XXXX = Modbus CRC-16 dari <data_object> (hex uppercase, 4 char).
 */
MQTT_StatusTypeDef MQTT_PublishWithCRC(const char *topic,
                                       const char *data_object,
                                       int qos, int retain);

/* ── Incoming message handling ────────────────────────────────────────────── */
/*
 * Call from main loop when mqtt_data_ready == true.
 * Parses the "+QMTRECV: ..." line from rxBuffer.
 *
 * Returns true and fills topic_out / payload_out on success.
 * Clears mqtt_data_ready after parsing.
 *
 * Limitation: payload must not contain a literal "\r\n" sequence.
 */
bool MQTT_ProcessIncoming(char *topic_out,   size_t topic_size,
                          char *payload_out, size_t payload_size);
MQTT_StatusTypeDef mqtt_read_time(void);

#endif /* INC_MQTT_H_ */
