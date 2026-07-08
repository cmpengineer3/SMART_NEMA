/*
 * config.h
 *
 *  Konfigurasi broker MQTT, topic, dan parameter device untuk Smart-KWH.
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "mqtt.h"

/* ── Identitas device ─────────────────────────────────────────────────────── */
#define uid "KWH01310000022"
#define LATITUDE   -7.276598
#define LONGITUDE  112.795616

/* ── Broker / kredensial (definisi di config.c) ──────────────────────────── */
extern char broker_host[64];
extern int  broker_port;
extern char client_id[64];
extern char username[64];
extern char password[64];

extern char mqtt_topic_pub[128];
extern char mqtt_topic_sub[128];

extern char mac_cco[24];

/* ── Timing ───────────────────────────────────────────────────────────────── */
extern uint32_t read_interval;   /* interval baca ADE7880 (ms)   */
extern uint32_t send_interval;   /* interval publish MQTT (ms)   */

extern int resend_count;         /* counter publish yang gagal   */

/* ── Header payload (opsional) ────────────────────────────────────────────── */
extern char header_x[64];
extern char header_y[64];

/* ── Waktu (di-update oleh mqtt_read_time via AT+QLTS=2) ──────────────────── */
extern int year;
extern int month;
extern int day;
extern int hour;
extern int minute;
extern int second;

/* Helper: bangun struct MQTT_Config dari variabel di atas */
MQTT_Config Config_GetMQTT(void);

#endif
