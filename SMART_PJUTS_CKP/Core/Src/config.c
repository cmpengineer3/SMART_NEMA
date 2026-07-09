/*
 * config.c
 *
 *  Created on: Sep 1, 2025
 *      Author: firza
 */

#include "config.h"
char broker_host[64] = "mqtt.caturmukti.com";
int  broker_port     = 1883;

char client_id[64]   = "PJU_TS";
char username[64]    = "ngadimin";
char password[64]    = "ngadimin#123";
//==============================================================================================================
// Konfigurasi MQTT Broker
//char broker_host[64] = "broker.hivemq.com";
//int  broker_port     = 1883;
//char client_id[64]   = "stm32Client";
//char username[64]    = "";  // kosong = tanpa auth
//char password[64]    = "";

//==============================================================================================================
// Topic MQTT
//char mqtt_topic_pubs[128] = "smartsiklon/000100000001/uplink";
//char mqtt_topic_subs[128] = "smartsiklon/000100000001/downlink";
//char cco_topic_reg[128] = "smartsiklon/cco/register";
//char sta_topic_reg[128] = "smartsiklon/000100000001/sta/register";

char mqtt_topic_pub[128] = "SIKLON/SMARTPJUTS/JKT/UID123456789/UPLINK";
char mqtt_topic_sub[128] = "SIKLON/SMARTPJUTS/JKT/UID123456789/DOWNLINK";

char mac_cco[24];
/* mqtt_msg_id removed — managed internally by mqtt.c */

/* ── Helper: pack config variables into MQTT_Config struct ────────────────── */
MQTT_Config Config_GetMQTT(void)
{
    MQTT_Config cfg;

    strncpy(cfg.broker_host, broker_host, sizeof(cfg.broker_host) - 1);
    cfg.broker_host[sizeof(cfg.broker_host) - 1] = '\0';

    cfg.broker_port = broker_port;

    strncpy(cfg.client_id, client_id, sizeof(cfg.client_id) - 1);
    cfg.client_id[sizeof(cfg.client_id) - 1] = '\0';

    strncpy(cfg.username, username, sizeof(cfg.username) - 1);
    cfg.username[sizeof(cfg.username) - 1] = '\0';

    strncpy(cfg.password, password, sizeof(cfg.password) - 1);
    cfg.password[sizeof(cfg.password) - 1] = '\0';

    return cfg;
}

//==============================================================================================================
char header_x[64]	 = "E";
char header_y[64]	 = "D";
//
//int  client_idx   = 0;
//char mqtt_topic_pub[96];
//char mqtt_topic_sub[96];
//int  mqtt_qos        = 1;
//int  mqtt_retain     = 0;

float batteryVoltage = 0.0f;
float batteryCurrent = 0.0f;
float batterySOC = 0.0f;
float deviceTemperature = 0.0f;
float batteryTemperature = 0.0f;
float loadVoltage = 0.0f;
float loadCurrent = 0.0f;
float loadPower = 0.0f;
float pvVolt = 0.0f;
float pvCurrent = 0.0f;
float pvPower = 0.0f;

// Tambahan untuk charge state
uint8_t chargeState = 0;
int loadState = 0;
int loadIntensity = 0;

/* ── Waktu (di-update oleh mqtt_read_time via AT+QLTS=2) ─────────────────── */
int year   = 0;
int month  = 0;
int day    = 0;
int hour   = 0;
int minute = 0;
int second = 0;

/* ── Timing ───────────────────────────────────────────────────────────────── */
uint32_t read_interval = 10000;   /* baca sensor tiap 10 detik */
uint32_t send_interval = 300000;   /* publish tiap 30 detik     */

int resend_count = 0;
