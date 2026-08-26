
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

void MQTT_Init(const MQTT_Config *cfg);

/* ── Utility ──────────────────────────────────────────────────────────────── */
uint16_t Modbus_CRC16(const unsigned char *buf, size_t len);


bool MQTT_ExtractDataObject(const char *payload, char *out, size_t out_size);

bool MQTT_VerifyPayloadCRC(const char *payload);

/* ── Network & session ────────────────────────────────────────────────────── */
MQTT_StatusTypeDef MQTT_CheckNetwork(void);
MQTT_StatusTypeDef MQTT_Open(void);
MQTT_StatusTypeDef MQTT_Connect(void);
MQTT_StatusTypeDef MQTT_Disconnect(void);
MQTT_StatusTypeDef MQTT_Close(void);

MQTT_StatusTypeDef MQTT_Reconnect(void);

/* ── Publish / Subscribe ──────────────────────────────────────────────────── */
MQTT_StatusTypeDef MQTT_Subscribe(const char *topic, int qos);
MQTT_StatusTypeDef MQTT_Publish(const char *topic, const char *payload,
                                int qos, int retain);

MQTT_StatusTypeDef MQTT_PublishWithCRC(const char *topic,
                                       const char *data_object,
                                       int qos, int retain);

/* ── Incoming message handling ────────────────────────────────────────────── */

bool MQTT_ProcessIncoming(char *topic_out,   size_t topic_size,
                          char *payload_out, size_t payload_size);
MQTT_StatusTypeDef mqtt_read_time(void);

#endif /* INC_MQTT_H_ */
