/*
 * mqtt.c (FIXED VERSION - Compatible with volatile buffers)
 *
 * PENTING: Pastikan ATE0 dijalankan di awal untuk matikan echo!
 */

#include "mqtt.h"
#include "ssd1306.h"
#include <stdio.h>

/* ============== CRC FUNCTION ============== */

uint16_t Modbus_CRC16(const unsigned char *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos];
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ============== HELPER FUNCTION ============== */

static MQTT_StatusTypeDef send_cmd_and_wait(const char *cmd,
                                            const char *urc_expected,
                                            uint32_t timeout_ok_ms,
                                            uint32_t timeout_urc_ms)
{
    UART_SendATCommand(cmd);

    if (UART_WaitFor_OK_Then_URC(urc_expected, timeout_ok_ms, timeout_urc_ms))
    {
        return MQTT_OK;
    }
    return MQTT_ERROR;
}

/* ============== NETWORK CHECK ============== */

MQTT_StatusTypeDef MQTT_CheckNetwork(void)
{
    // Set full functionality
    UART_SendATCommand("AT+CFUN=1");
    if (!UART_WaitForOK(5000))
    {
        return MQTT_ERROR;
    }
    HAL_Delay(500);

    // Cek registrasi jaringan
    UART_SendATCommand("AT+CEREG?");
    if (!UART_WaitForOK(5000))
    {
        return MQTT_ERROR;
    }

    // Cek apakah terdaftar - CAST volatile ke const char*
    ENTER_CRITICAL();
    bool registered = (strstr((const char*)rxBuffer, "+CEREG: 0,1") != NULL ||
                       strstr((const char*)rxBuffer, "+CEREG: 0,5") != NULL);
    EXIT_CRITICAL();

    if (!registered)
    {
        return MQTT_ERROR;
    }
    HAL_Delay(500);

    // Cek GPRS attach
    UART_SendATCommand("AT+CGATT?");
    if (!UART_WaitForOK(5000))
    {
        return MQTT_ERROR;
    }

    // CAST volatile ke const char*
    ENTER_CRITICAL();
    bool attached = (strstr((const char*)rxBuffer, "+CGATT: 1") != NULL);
    EXIT_CRITICAL();

    if (!attached)
    {
        UART_SendATCommand("AT+CGATT=1");
        if (!UART_WaitForOK(10000))
        {
            return MQTT_ERROR;
        }
        HAL_Delay(2000);
    }

    // Cek sinyal
    UART_SendATCommand("AT+CSQ");
    if (!UART_WaitForOK(3000))
    {
        return MQTT_ERROR;
    }

    return MQTT_OK;
}

/* ============== MQTT OPEN ============== */

MQTT_StatusTypeDef MQTT_Open(void)
{
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+QMTOPEN=0,\"%s\",%d", broker_host, broker_port);
    return send_cmd_and_wait(cmd, "+QMTOPEN: 0,0", 5000, 25000);
}

/* ============== MQTT CONNECT ============== */

MQTT_StatusTypeDef MQTT_Connect(void)
{
    char cmd[256];
    if (username[0] != '\0')
    {
        snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"",
                 client_id, username, password);
    }
    else
    {
        snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\"", client_id);
    }

    return send_cmd_and_wait(cmd, "+QMTCONN: 0,0,0", 5000, 25000);
}

/* ============== MQTT SUBSCRIBE ============== */

MQTT_StatusTypeDef MQTT_Subscribe(const char *topic, int qos)
{
    char cmd[256];
    char urc[64];

    snprintf(cmd, sizeof(cmd), "AT+QMTSUB=0,%d,\"%s\",%d", mqtt_msg_id, topic, qos);
    snprintf(urc, sizeof(urc), "+QMTSUB: 0,%d,0", mqtt_msg_id);

    return send_cmd_and_wait(cmd, urc, 5000, 15000);
}

/* ============== MQTT PUBLISH ============== */

MQTT_StatusTypeDef MQTT_Publish(const char *topic, const char *payload, int qos, int retain)
{
    char cmd[512];
    int payload_len = (int)strlen(payload);

    for (int retry = 0; retry < 3; retry++)
    {
        /* ------------------------------------------------
         * STEP 1: Paksa matikan echo setiap kali sebelum
         * publish. ATE0 dikirim via SendATCommand yang
         * otomatis clear buffer dulu.
         * ------------------------------------------------ */
        UART_SendATCommand("ATE0");
        HAL_Delay(200);
        // Pastikan OK diterima. Jika tidak, tetap lanjut
        // (mungkin echo sudah mati dari sebelumnya)
        UART_ClearBuffer();

        /* ------------------------------------------------
         * STEP 2: Kirim AT+QMTPUBEX
         * Pakai SendATCommand agar buffer clear dulu,
         * sehingga tidak ada sisa data sebelumnya
         * ------------------------------------------------ */
        snprintf(cmd, sizeof(cmd),
                 "AT+QMTPUBEX=0,%d,%d,%d,\"%s\",%d",
                 mqtt_msg_id, qos, retain, topic, payload_len);
        UART_SendATCommand(cmd);  // ← clear buffer + kirim

        /* ------------------------------------------------
         * STEP 3: Tunggu prompt ">"
         * Karena echo sudah dimatikan di STEP 1,
         * ">" yang diterima pasti prompt asli dari modem
         * ------------------------------------------------ */
        if (!UART_WaitForPrompt(">", 5000))
        {
            HAL_Delay(200);
            continue;
        }

        /* ------------------------------------------------
         * STEP 4: Clear buffer lagi setelah dapat prompt,
         * lalu kirim payload murni
         * ------------------------------------------------ */
        UART_ClearBuffer();
        HAL_Delay(30);

        HAL_UART_Transmit(&huart1, (uint8_t*)payload,
                          (uint16_t)strlen(payload), 5000);
        HAL_Delay(30);

        /* ------------------------------------------------
         * STEP 5: Kirim CTRL+Z untuk submit payload
         * ------------------------------------------------ */
        uint8_t ctrlz = 0x1A;
        HAL_UART_Transmit(&huart1, &ctrlz, 1, 3000);

        /* ------------------------------------------------
         * STEP 6: Tunggu konfirmasi OK + URC dari modem
         * ------------------------------------------------ */
        if (UART_WaitFor_OK_Then_URC("+QMTPUBEX:", 10000, 10000))
        {
            return MQTT_OK;
        }

        HAL_Delay(500);
    }

    return MQTT_ERROR;
}

/* ============== MQTT DISCONNECT & CLOSE ============== */

MQTT_StatusTypeDef MQTT_Disconnect(void)
{
    return send_cmd_and_wait("AT+QMTDISC=0", "+QMTDISC: 0,0", 5000, 10000);
}

MQTT_StatusTypeDef MQTT_Close(void)
{
    return send_cmd_and_wait("AT+QMTCLOSE=0", "+QMTCLOSE: 0,0", 5000, 10000);
}

MQTT_StatusTypeDef MQTT_Reconnect(void)
{
    // Tutup koneksi lama dulu
    MQTT_Disconnect();
    HAL_Delay(1000);
    MQTT_Close();
    HAL_Delay(2000);

    // Buka ulang
    if (MQTT_Open() != MQTT_OK) return MQTT_ERROR;
    HAL_Delay(500);

    if (MQTT_Connect() != MQTT_OK) return MQTT_ERROR;
    HAL_Delay(500);

    // Subscribe ulang
    if (MQTT_Subscribe(mqtt_topic_sub, 1) != MQTT_OK) return MQTT_ERROR;

    mqtt_disconnected = false;
    return MQTT_OK;
}

/* ============== MQTT PROCESS INCOMING ============== */

//bool MQTT_ProcessIncoming(char *topic, size_t topic_size, char *payload, size_t payload_size)
//{
//    if (UART_HasMqttMessage())
//    {
//        UART_GetMqttMessage(topic, topic_size, payload, payload_size);
//        return true;
//    }
//    return false;
//}
