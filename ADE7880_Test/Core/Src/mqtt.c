#include "mqtt.h"
#include "main.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ── Internal state ───────────────────────────────────────────────────────── */
static MQTT_Config s_cfg   = {0};
static uint16_t    s_msg_id = 1;

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Init                                                                       */
/* ─────────────────────────────────────────────────────────────────────────── */

void MQTT_Init(const MQTT_Config *cfg)
{
    if (cfg == NULL) return;
    s_cfg   = *cfg;
    s_msg_id = 1;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Utility: Modbus CRC-16                                                    */
/* ─────────────────────────────────────────────────────────────────────────── */

uint16_t Modbus_CRC16(const unsigned char *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos];
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else              { crc >>= 1; }
        }
    }
    return crc;
}

/* Konversi CRC16 menjadi 4-char hex uppercase (sesuai format dokumentasi) */
static void crc16_to_hex(uint16_t crc, char out[5])
{
    static const char HEX[] = "0123456789ABCDEF";
    out[0] = HEX[(crc >> 12) & 0xF];
    out[1] = HEX[(crc >>  8) & 0xF];
    out[2] = HEX[(crc >>  4) & 0xF];
    out[3] = HEX[(crc      ) & 0xF];
    out[4] = '\0';
}

bool MQTT_ExtractDataObject(const char *payload, char *out, size_t out_size)
{
    if (payload == NULL || out == NULL || out_size == 0) return false;

    const char *p = strstr(payload, "\"DATA\":");
    if (p == NULL) return false;
    p += 7;                              /* skip "DATA": */
    while (*p == ' ') p++;
    if (*p != '{') return false;

    const char *start = p;
    int depth = 0;
    const char *end = NULL;

    while (*p != '\0')
    {
        if      (*p == '{') depth++;
        else if (*p == '}')
        {
            depth--;
            if (depth == 0) { end = p; break; }
        }
        p++;
    }

    if (end == NULL) return false;

    size_t len = (size_t)(end - start) + 1;
    if (len >= out_size) return false;

    memcpy(out, start, len);
    out[len] = '\0';
    return true;
}

bool MQTT_VerifyPayloadCRC(const char *payload)
{
    if (payload == NULL) return false;

    const char *p = strstr(payload, "\"CRC\":");
    if (p == NULL) return false;
    p += 6;                              /* skip "CRC": */
    while (*p == ' ') p++;
    if (*p != '\"') return false;
    p++;

    char crc_str[8] = {0};
    size_t i = 0;
    while (*p != '\0' && *p != '\"' && i < sizeof(crc_str) - 1)
        crc_str[i++] = *p++;
    if (i == 0 || *p != '\"') return false;

    uint16_t crc_recv = (uint16_t)strtoul(crc_str, NULL, 16);

    char data_obj[512];
    if (!MQTT_ExtractDataObject(payload, data_obj, sizeof(data_obj)))
        return false;

    uint16_t crc_calc = Modbus_CRC16((const unsigned char *)data_obj,
                                     strlen(data_obj));
    return (crc_calc == crc_recv);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Internal helper: send AT command and wait for OK + URC                    */
/* ─────────────────────────────────────────────────────────────────────────── */

static MQTT_StatusTypeDef send_and_wait(const char *cmd,
                                        const char *urc,
                                        uint32_t    timeout_ok_ms,
                                        uint32_t    timeout_urc_ms)
{
    UART_SendATCommand(cmd);
    return (UART_WaitFor_OK_Then_URC(urc, timeout_ok_ms, timeout_urc_ms))
           ? MQTT_OK : MQTT_ERROR;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Network check                                                              */
/* ─────────────────────────────────────────────────────────────────────────── */

MQTT_StatusTypeDef MQTT_CheckNetwork(void)
{
    /* Sama seperti Test_Modem_MQTT: CFUN=1 memastikan radio stack aktif.
     * WatchdogRefresh di uart.c mencegah IWDG reset selama tunggu OK. */
    UART_SendATCommand("AT+CFUN=1");
    if (!UART_WaitForOK(5000)) return MQTT_ERROR;
    HAL_Delay(500);

    /* Cek registrasi jaringan */
    UART_SendATCommand("AT+CEREG?");
    if (!UART_WaitForOK(5000)) return MQTT_ERROR;

    ENTER_CRITICAL();
    bool registered = (strstr((const char *)rxBuffer, "+CEREG: 0,1") != NULL ||
                       strstr((const char *)rxBuffer, "+CEREG: 0,5") != NULL);
    EXIT_CRITICAL();
    if (!registered) return MQTT_ERROR;

    HAL_Delay(500);

    /* Cek attachment packet data */
    UART_SendATCommand("AT+CGATT?");
    if (!UART_WaitForOK(5000)) return MQTT_ERROR;

    ENTER_CRITICAL();
    bool attached = (strstr((const char *)rxBuffer, "+CGATT: 1") != NULL);
    EXIT_CRITICAL();

    if (!attached)
    {
        UART_SendATCommand("AT+CGATT=1");
        if (!UART_WaitForOK(10000)) return MQTT_ERROR;
        HAL_Delay(2000);
    }

    /* Cek kualitas sinyal */
    UART_SendATCommand("AT+CSQ");
    if (!UART_WaitForOK(3000)) return MQTT_ERROR;

    return MQTT_OK;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  MQTT Open                                                                  */
/* ─────────────────────────────────────────────────────────────────────────── */

MQTT_StatusTypeDef MQTT_Open(void)
{
    static char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+QMTOPEN=0,\"%s\",%d",
             s_cfg.broker_host, s_cfg.broker_port);
    /* URC timeout 90s — sama seperti Test_Modem_MQTT.
     * UART_WatchdogRefresh() di uart.c refresh IWDG setiap 5s sehingga
     * IWDG (~32s) tidak akan fire meskipun menunggu sampai 90s. */
    return send_and_wait(cmd, "+QMTOPEN: 0,0", 5000, 90000);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  MQTT Connect                                                               */
/* ─────────────────────────────────────────────────────────────────────────── */

MQTT_StatusTypeDef MQTT_Connect(void)
{
    static char cmd[256];
    if (s_cfg.username[0] != '\0')
    {
        snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"",
                 s_cfg.client_id, s_cfg.username, s_cfg.password);
    }
    else
    {
        snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\"", s_cfg.client_id);
    }
    return send_and_wait(cmd, "+QMTCONN: 0,0,0", 5000, 25000);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  MQTT Subscribe                                                             */
/* ─────────────────────────────────────────────────────────────────────────── */

MQTT_StatusTypeDef MQTT_Subscribe(const char *topic, int qos)
{
    static char cmd[256];
    static char urc[64];

    snprintf(cmd, sizeof(cmd), "AT+QMTSUB=0,%d,\"%s\",%d", s_msg_id, topic, qos);
    snprintf(urc, sizeof(urc), "+QMTSUB: 0,%d,0", s_msg_id);

    MQTT_StatusTypeDef result = send_and_wait(cmd, urc, 5000, 15000);
    s_msg_id++;
    return result;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  MQTT Publish                                                               */
/* ─────────────────────────────────────────────────────────────────────────── */

MQTT_StatusTypeDef MQTT_Publish(const char *topic, const char *payload,
                                int qos, int retain)
{
    if (topic == NULL || payload == NULL) return MQTT_ERROR;

    int  payload_len = (int)strlen(payload);
    static char cmd[512];

    for (int retry = 0; retry < 3; retry++)
    {
        UART_ClearBuffer();

        snprintf(cmd, sizeof(cmd),
                 "AT+QMTPUBEX=0,%d,%d,%d,\"%s\",%d",
                 s_msg_id, qos, retain, topic, payload_len);
        UART_SendATCommand(cmd);

        if (!UART_WaitForPrompt(">", 5000))
        {
            HAL_Delay(300);
            continue;
        }

        UART_ClearBuffer();
        HAL_Delay(30);

        UART_TransmitRaw((const uint8_t *)payload, (uint16_t)payload_len);
        HAL_Delay(50);

        static char urc[32];
        snprintf(urc, sizeof(urc), "+QMTPUBEX: 0,%d,0", s_msg_id);

        if (UART_WaitFor_OK_Then_URC(urc, 10000, 15000))
        {
            s_msg_id++;
            return MQTT_OK;
        }

        HAL_Delay(500);
    }

    return MQTT_ERROR;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  MQTT Publish with auto-CRC                                                */
/* ─────────────────────────────────────────────────────────────────────────── */

MQTT_StatusTypeDef MQTT_PublishWithCRC(const char *topic,
                                       const char *data_object,
                                       int qos, int retain)
{
    if (topic == NULL || data_object == NULL) return MQTT_ERROR;

    uint16_t crc = Modbus_CRC16((const unsigned char *)data_object,
                                strlen(data_object));
    char hex[5];
    crc16_to_hex(crc, hex);

    static char full_payload[1024];
    int n = snprintf(full_payload, sizeof(full_payload),
                     "{\"CRC\":\"%s\",\"DATA\":%s}", hex, data_object);
    if (n < 0 || n >= (int)sizeof(full_payload)) return MQTT_ERROR;

    return MQTT_Publish(topic, full_payload, qos, retain);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  MQTT Disconnect / Close / Reconnect                                       */
/* ─────────────────────────────────────────────────────────────────────────── */

MQTT_StatusTypeDef MQTT_Disconnect(void)
{
    return send_and_wait("AT+QMTDISC=0", "+QMTDISC: 0,0", 5000, 10000);
}

MQTT_StatusTypeDef MQTT_Close(void)
{
    return send_and_wait("AT+QMTCLOSE=0", "+QMTCLOSE: 0,0", 5000, 10000);
}

MQTT_StatusTypeDef MQTT_Reconnect(void)
{
    MQTT_Disconnect();
    HAL_Delay(1000);
    MQTT_Close();
    HAL_Delay(2000);

    if (MQTT_Open()    != MQTT_OK) return MQTT_ERROR;
    HAL_Delay(500);
    if (MQTT_Connect() != MQTT_OK) return MQTT_ERROR;

    mqtt_disconnected = false;
    return MQTT_OK;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Incoming message parsing                                                   */
/* ─────────────────────────────────────────────────────────────────────────── */

bool MQTT_ProcessIncoming(char *topic_out,   size_t topic_size,
                          char *payload_out, size_t payload_size)
{
    if (!mqtt_data_ready) return false;
    if (topic_out == NULL || payload_out == NULL) return false;

    static char local[RX_BUFFER_SIZE];
    ENTER_CRITICAL();
    strncpy(local, (const char *)rxBuffer, RX_BUFFER_SIZE - 1);
    local[RX_BUFFER_SIZE - 1] = '\0';
    mqtt_data_ready = false;
    EXIT_CRITICAL();

    char *p = strstr(local, "+QMTRECV:");
    if (p == NULL) return false;

    int commas = 0;
    while (*p && commas < 2)
    {
        if (*p == ',') commas++;
        p++;
    }
    while (*p == ' ') p++;

    if (*p != '"') return false;
    p++;

    char *topic_end = strchr(p, '"');
    if (topic_end == NULL) return false;

    size_t topic_len = (size_t)(topic_end - p);
    if (topic_len >= topic_size) topic_len = topic_size - 1;
    strncpy(topic_out, p, topic_len);
    topic_out[topic_len] = '\0';

    p = topic_end + 1;
    if (*p == ',') p++;
    while (*p == ' ') p++;

    if (*p != '"') return false;
    p++;

    char *payload_start = p;

    char *line_end = strpbrk(payload_start, "\r\n");
    if (line_end == NULL) line_end = payload_start + strlen(payload_start);

    char *payload_end = line_end;
    while (payload_end > payload_start && *(payload_end - 1) != '"')
        payload_end--;
    if (payload_end > payload_start) payload_end--;

    size_t payload_len = (size_t)(payload_end - payload_start);
    if (payload_len >= payload_size) payload_len = payload_size - 1;
    strncpy(payload_out, payload_start, payload_len);
    payload_out[payload_len] = '\0';

    return true;
}

/* Variabel waktu global (defined di config.c). Di-extern lokal di sini agar
 * mqtt.c tetap tidak perlu include config.h. */
extern int year, month, day, hour, minute, second;

MQTT_StatusTypeDef mqtt_read_time(void)
{
    /* AT+QLTS=2 → +QLTS: "YYYY/MM/DD,hh:mm:ss+ZZ,DST"
     * (network time; butuh registrasi CEREG dulu).                        */
    UART_SendATCommand("AT+QLTS=2");
    if (!UART_WaitForOK(3000)) return MQTT_ERROR;

    /* Salin rxBuffer ke lokal dulu — rxBuffer volatile & bisa berubah oleh
     * ISR di tengah parsing kalau URC lain masuk.                          */
    static char local[RX_BUFFER_SIZE];
    ENTER_CRITICAL();
    strncpy(local, (const char *)rxBuffer, RX_BUFFER_SIZE - 1);
    local[RX_BUFFER_SIZE - 1] = '\0';
    EXIT_CRITICAL();

    char *p = strstr(local, "+QLTS:");
    if (p == NULL) return MQTT_ERROR;

    int yr, mo, dy, hr, mi, sc;
    if (sscanf(p, "+QLTS: \"%d/%d/%d,%d:%d:%d",
               &yr, &mo, &dy, &hr, &mi, &sc) == 6)
    {
        year   = yr;
        month  = mo;
        day    = dy;
        hour   = hr;
        minute = mi;
        second = sc;
        return MQTT_OK;
    }
    return MQTT_ERROR;
}
