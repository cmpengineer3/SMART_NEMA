/*
 * mqtt.h (FIXED VERSION)
 */

#ifndef INC_MQTT_H_
#define INC_MQTT_H_

#include "uart.h"

typedef enum {
    MQTT_OK = 0,
    MQTT_ERROR = 1
} MQTT_StatusTypeDef;

// CRC function
extern uint16_t Modbus_CRC16(const unsigned char *buf, size_t len);

// Fungsi koneksi dasar
MQTT_StatusTypeDef MQTT_CheckNetwork(void);
MQTT_StatusTypeDef MQTT_Open(void);
MQTT_StatusTypeDef MQTT_Connect(void);
MQTT_StatusTypeDef MQTT_Subscribe(const char *topic, int qos);
MQTT_StatusTypeDef MQTT_Publish(const char *topic, const char *payload, int qos, int retain);
MQTT_StatusTypeDef MQTT_Disconnect(void);
MQTT_StatusTypeDef MQTT_Close(void);
MQTT_StatusTypeDef MQTT_Reconnect(void);

// Process incoming MQTT messages (panggil di main loop)
bool MQTT_ProcessIncoming(char *topic, size_t topic_size, char *payload, size_t payload_size);

#endif /* INC_MQTT_H_ */
