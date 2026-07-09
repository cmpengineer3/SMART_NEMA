/*
 * config.c
 *
 *  Konfigurasi broker MQTT + topic + parameter device Smart-KWH.
 *
 *  ⚠️ CATATAN KEAMANAN:
 *  Jangan commit kredensial broker asli (host/username/password) ke repo
 *  publik. Untuk pengetesan, contoh di bawah memakai broker publik HiveMQ
 *  tanpa auth. Ganti ke broker produksi hanya di device, atau simpan di file
 *  terpisah yang di-.gitignore.
 */

#include "config.h"
#include <stdio.h>

/* ── Konfigurasi MQTT Broker (contoh: broker publik untuk tes) ────────────── */
//char broker_host[64] = "broker.hivemq.com";
//int  broker_port     = 1883;
//char client_id[64]   = "stm32KwhClient";
//char username[64]    = "";   // kosong = tanpa auth
//char password[64]    = "";

// Untuk broker produksi (isi di device, JANGAN commit ke repo publik):
 char broker_host[64] = "mqtt.caturmukti.com";
 int  broker_port     = 1883;
 char client_id[64]   = "SMART_CCO";
 char username[64]    = "ngadimin";
 char password[64]    = "ngadimin#123";
// char username[64]    = "admin";
// char password[64]    = "xK645P7Np6LR";


/* ── Topic MQTT ───────────────────────────────────────────────────────────── */
char mqtt_regis_pub[128] = "SIKLON/SMARTPJU/CCO/REGISTER";
char mqtt_topic_pub[128];
char mqtt_topic_sub[128];


//sprintf(mqtt_topic_pub, "SIKLON/SMARTPJU/JKT/&s/UPLINK/CCO", UID);
//sprintf(mqtt_topic_sub, "SIKLON/SMARTPJU/JKT/&s/DOWNLINK", UID);


//char mqtt_topic_pub[128] = "SIKLON/SMARTPJU/JKT/KWH0426DEMO0/UPLINK/CCO";
//char mqtt_topic_sub[128] = "SIKLON/SMARTPJU/JKT/KWH0426DEMO0/DOWNLINK";

char mac_cco[24];

/* ── Helper: pack config ke struct MQTT_Config ────────────────────────────── */
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

/* ── Header payload (opsional) ────────────────────────────────────────────── */
char header_x[64] = "E";
char header_y[64] = "D";

/* ── Waktu (di-update oleh mqtt_read_time via AT+QLTS=2) ──────────────────── */
int year   = 0;
int month  = 0;
int day    = 0;
int hour   = 0;
int minute = 0;
int second = 0;

/* ── Timing ───────────────────────────────────────────────────────────────── */
uint32_t read_interval = 2000;    /* baca ADE7880 tiap 10 detik   */
uint32_t send_interval = 30000;    /* publish MQTT tiap 60 detik   */

int resend_count = 0;
