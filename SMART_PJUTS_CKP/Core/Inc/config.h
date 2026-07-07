/*
 * config.h
 *
 *  Created on: Sep 1, 2025
 *      Author: firza
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "mqtt.h"

#define uid "UID123456789"		//UID DEVICE=====================================
extern char broker_host[64];
extern int  broker_port;
extern char client_id[64];
extern char username[64];
extern char password[64];

extern char mqtt_topic_pub[128];
extern char mqtt_topic_sub[128];

extern char mac_cco[24];

/* ── Timing ───────────────────────────────────────────────────────────────── */
extern uint32_t read_interval;   /* interval baca sensor SRNE (ms) */
extern uint32_t send_interval;   /* interval publish MQTT (ms)     */

extern int resend_count;         /* counter publish yang gagal     */

extern float batteryVoltage;
extern float batteryCurrent;
extern float batterySOC;
extern float deviceTemperature;
extern float batteryTemperature;
extern float loadVoltage;
extern float loadCurrent;
extern float loadPower;
extern float pvVolt;
extern float pvCurrent;
extern float pvPower;

// Tambahan untuk charge state
extern uint8_t chargeState;
extern int loadState;
extern int loadIntensity;

extern char header_x[64];
extern char header_y[64];
extern int year;
extern int month;
extern int day;
extern int hour;
extern int minute;
extern int second;
/* mqtt_msg_id removed — managed internally by mqtt.c (auto-increment) */

/* Helper: build MQTT_Config struct from config variables above */
MQTT_Config Config_GetMQTT(void);

#endif
