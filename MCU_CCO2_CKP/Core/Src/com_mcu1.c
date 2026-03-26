/*
 * com_mcu1.c
 *
 *  Created on: Jan 27, 2026
 *      Author: firza
 */


/*
 * mcu1_request.c
 *
 * Code untuk MCU2 (MQTT Gateway) - Request data dari MCU1
 *
 * Flow:
 * 1. MQTT kirim {"H":"R"} → MCU2 mulai request
 * 2. MCU2 request jumlah node: "REQ:CNT\n" → dapat "CNT:5\n"
 * 3. MCU2 request node satu per satu: "REQ:0\n", "REQ:1\n", dst
 * 4. Setiap dapat response → kirim ke MQTT
 */

#include "uart.h"
#include "mqtt.h"
#include "config.h"
#include "com_mcu1.h"
#include "ssd1306.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ============================================================================
//                    KONFIGURASI
// ============================================================================

#define MCU1_UART           huart2          // UART untuk komunikasi dengan MCU1
#define MCU1_TIMEOUT_MS     10000            // Timeout tunggu response dari MCU1
#define MCU1_RETRY_COUNT    2               // Retry jika gagal

// ============================================================================
//                    BUFFER DAN VARIABEL
// ============================================================================

// Buffer untuk terima data dari MCU1
#define MCU1_RX_BUFFER_SIZE  512
volatile char mcu1_rx_buffer[MCU1_RX_BUFFER_SIZE];
char debug[MCU1_RX_BUFFER_SIZE];
volatile uint16_t mcu1_rx_index = 0;
volatile bool mcu1_data_ready = false;

volatile uint32_t test_send_count = 0;       // Berapa kali kirim
//char test_debug[64] = {0};
volatile char jml_node [10];
// Variabel untuk Live Expression monitoring
volatile uint16_t node_total_count = 0;      // Jumlah total node dari MCU1
volatile uint16_t node_current_index = 0;    // Index node yang sedang diproses
volatile uint16_t node_success_count = 0;    // Jumlah node yang berhasil dikirim ke MQTT
volatile uint16_t node_fail_count = 0;       // Jumlah node yang gagal
volatile bool is_requesting = false;         // Flag sedang dalam proses request

// Byte untuk interrupt
static uint8_t mcu1_rx_byte;

extern UART_HandleTypeDef huart2;  // UART untuk MCU1

// ============================================================================
//                    INISIALISASI
// ============================================================================

void MCU1_Request_Init(void)
{
    memset((void*)mcu1_rx_buffer, 0, MCU1_RX_BUFFER_SIZE);
    mcu1_rx_index = 0;
    mcu1_data_ready = false;

    node_total_count = 0;
    node_current_index = 0;
    node_success_count = 0;
    node_fail_count = 0;
    is_requesting = false;

    // Start interrupt untuk UART3 (komunikasi dengan MCU1)
    HAL_UART_Receive_IT(&huart2, &mcu1_rx_byte, 1);
}

// ============================================================================
//                    CALLBACK UART3 (Terima dari MCU1)
// ============================================================================
// CATATAN: Tambahkan ke HAL_UART_RxCpltCallback yang sudah ada

void MCU1_UART_RxCallback(void)
{
    // CLEAR buffer lama jika ada data baru masuk dan buffer sebelumnya sudah diproses
    // atau jika ini adalah awal data baru (karakter '{' atau 'C' untuk "CNT:")
    if (mcu1_rx_byte == '{' || (mcu1_rx_byte == 'C' && mcu1_rx_index == 0))
    {
        // Clear buffer sebelum data baru
//        memset((void*)mcu1_rx_buffer, 0, MCU1_RX_BUFFER_SIZE);
        mcu1_rx_index = 0;
        mcu1_data_ready = false;
    }

    // Simpan byte
    if (mcu1_rx_index < MCU1_RX_BUFFER_SIZE - 1)
    {
        mcu1_rx_buffer[mcu1_rx_index] = mcu1_rx_byte;
        mcu1_rx_index++;
        mcu1_rx_buffer[mcu1_rx_index] = '\0';

        // Deteksi akhir pesan (newline)
        if (mcu1_rx_byte == '\n')
        {
            mcu1_data_ready = true;
        }
    }
    else
    {
        // Buffer overflow - reset
        mcu1_rx_index = 0;
        mcu1_rx_buffer[0] = '\0';
    }

    // Re-enable interrupt
    HAL_UART_Receive_IT(&huart2, &mcu1_rx_byte, 1);
}

// ============================================================================
//                    CLEAR BUFFER MCU1
// ============================================================================

void MCU1_ClearBuffer(void)
{
    __disable_irq();
    memset((void*)mcu1_rx_buffer, 0, MCU1_RX_BUFFER_SIZE);
    mcu1_rx_index = 0;
    mcu1_data_ready = false;
    __enable_irq();
}

// ============================================================================
//                    KIRIM REQUEST KE MCU1
// ============================================================================

// Kirim request dan tunggu response
bool MCU1_SendRequest(const char *request, uint32_t timeout_ms)
{
    // Clear buffer dulu
    MCU1_ClearBuffer();

    // Kirim request
    char buf[32];
    snprintf(buf, sizeof(buf), "%s\n", request);
    snprintf(debug,sizeof(debug), "mau coba req");
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 1000);

    // Tunggu response
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (mcu1_data_ready)
        {
            return true;
        }
        HAL_Delay(10);
    }

    return false;  // Timeout
}

// ============================================================================
//                    REQUEST JUMLAH NODE
// ============================================================================

bool MCU1_RequestNodeCount(void)
{
    for (int retry = 0; retry < MCU1_RETRY_COUNT; retry++)
    {
        snprintf(debug,sizeof(debug), "coba itung");
        if (MCU1_SendRequest("REQ:CNT", MCU1_TIMEOUT_MS))
        {
            // Parse response "CNT:X"
            snprintf(debug,sizeof(debug), "REQ:CNT");
            char *ptr = strstr((const char*)mcu1_rx_buffer, "CNT:");
            if (ptr != NULL)
            {
                node_total_count = (uint16_t)atoi(ptr + 4);
                return true;
            }
        }
        HAL_Delay(200);
    }

    return false;
}

// ============================================================================
//                    REQUEST DATA NODE BY INDEX
// ============================================================================

bool MCU1_RequestNodeData(uint16_t index, char *json_out, size_t json_size)
{
    char request[16];
    snprintf(request, sizeof(request), "REQ:%u", index);

    for (int retry = 0; retry < MCU1_RETRY_COUNT; retry++)
    {
        if (MCU1_SendRequest(request, MCU1_TIMEOUT_MS))
        {
            // Cek apakah response valid (dimulai dengan '{')
            if (mcu1_rx_buffer[0] == '{')
            {
                // Copy response ke output
                strncpy(json_out, (const char*)mcu1_rx_buffer, json_size - 1);
                json_out[json_size - 1] = '\0';

                // Hapus newline di akhir jika ada
                size_t len = strlen(json_out);
                if (len > 0 && (json_out[len-1] == '\n' || json_out[len-1] == '\r'))
                {
                    json_out[len-1] = '\0';
                }
                if (len > 1 && (json_out[len-2] == '\n' || json_out[len-2] == '\r'))
                {
                    json_out[len-2] = '\0';
                }

                return true;
            }
            // Cek error response
            else if (strstr((const char*)mcu1_rx_buffer, "ERR:") != NULL)
            {
                return false;
            }
        }
        HAL_Delay(200);
    }

    return false;
}

bool MCU1_RegisNodeData(uint16_t index, char *json_out, size_t json_size)
{
    char request[16];
    snprintf(request, sizeof(request), "REG:%u", index);

    for (int retry = 0; retry < MCU1_RETRY_COUNT; retry++)
    {
        if (MCU1_SendRequest(request, MCU1_TIMEOUT_MS))
        {
            // Cek apakah response valid (dimulai dengan '{')
            if (mcu1_rx_buffer[0] == '{')
            {
                // Copy response ke output
                strncpy(json_out, (const char*)mcu1_rx_buffer, json_size - 1);
                json_out[json_size - 1] = '\0';

                // Hapus newline di akhir jika ada
                size_t len = strlen(json_out);
                if (len > 0 && (json_out[len-1] == '\n' || json_out[len-1] == '\r'))
                {
                    json_out[len-1] = '\0';
                }
                if (len > 1 && (json_out[len-2] == '\n' || json_out[len-2] == '\r'))
                {
                    json_out[len-2] = '\0';
                }

                return true;
            }
            // Cek error response
            else if (strstr((const char*)mcu1_rx_buffer, "ERR:") != NULL)
            {
                return false;
            }
        }
        HAL_Delay(200);
    }

    return false;
}

bool MCU2_RequestMacCCO(char *mac_out, size_t mac_size)
{
    char request[] = "MAC_CCO";

    // Kirim request ke MCU1 dan tunggu response
    if (!MCU1_SendRequest(request, MCU1_TIMEOUT_MS))
        return false;

    // -------------------------------------------------------
    // BUG FIX #1: Validasi header JSON menggunakan panjang
    // string yang BENAR (10 karakter, bukan 8).
    // String "{\"H\":\"RC\"" = { " H " : " R C " = 10 char.
    // Sebelumnya strncmp(..., 8) hanya cek sampai {"H":"R
    // sehingga validasi tidak bekerja dengan benar.
    // -------------------------------------------------------
    const char *expected_header = "{\"H\":\"RC\"";
    if (strncmp((char*)mcu1_rx_buffer, expected_header, strlen(expected_header)) != 0)
        return false;

    // Cari field "CCO"
    char *p = strstr((char*)mcu1_rx_buffer, "\"CCO\":\"");
    if (!p)
        return false;

    // Geser pointer ke awal nilai MAC (lewati "CCO":")
    p += strlen("\"CCO\":\"");

    // Cari penutup tanda kutip
    char *end = strchr(p, '"');
    if (!end)
        return false;

    size_t len = end - p;
    if (len == 0 || len >= mac_size)
        return false;

    strncpy(mac_out, p, len);
    mac_out[len] = '\0';

    // -------------------------------------------------------
    // BUG FIX #2: Publish MAC CCO ke topic cco_topic_reg.
    // Sebelumnya fungsi ini hanya mengisi mac_out tapi TIDAK
    // pernah mengirim data ke MQTT — data hilang begitu saja.
    // -------------------------------------------------------
    char cco_payload[256];
    snprintf(cco_payload, sizeof(cco_payload),
             "{\"H\":\"RC\",\"CCO\":\"%s\"}", mac_out);

    size_t payload_len = strlen(cco_payload);
    uint16_t crc = Modbus_CRC16((const unsigned char*)cco_payload, payload_len);

    char cco_mqtt_msg[320];
    snprintf(cco_mqtt_msg, sizeof(cco_mqtt_msg),
             "{\"CRC\":\"%04X\",\"DATA\":%s}", crc, cco_payload);

    MQTT_Publish(cco_topic_reg, cco_mqtt_msg, 1, 0);

    return true;
}

// ============================================================================
//                    PROSES REQUEST SEMUA NODE (Dipanggil saat dapat "H":"R")
// ============================================================================

void MCU2_RequestAllNodes(void)
{
    // Set flag requesting
    is_requesting = true;
    node_success_count = 0;
    node_fail_count = 0;
    node_current_index = 0;

    // -----------------------------------------
    // Step 1: Request jumlah node
    // -----------------------------------------
    if (!MCU1_RequestNodeCount())
    {
        // Gagal dapat jumlah node
        is_requesting = false;
//        snprintf(debug,sizeof(debug),"gagal counting");
        return;
    }

    // Jika tidak ada node
    if (node_total_count == 0)
    {
        is_requesting = false;
        snprintf(debug,sizeof(debug),"dikirim 0");
        return;
    }

    // -----------------------------------------
    // Step 2: Request setiap node dan kirim ke MQTT
    // -----------------------------------------
    char node_json[300];
    char mqtt_payload[512];

    for (uint16_t i = 0; i < node_total_count; i++)
    {
        node_current_index = i;

        // Request data node
        if (MCU1_RequestNodeData(i, node_json, sizeof(node_json)))
        {
            // Buat payload MQTT dengan CRC
            // Format: {"CRC":"XXXX","DATA":{...node data...}}
            size_t len = strlen(node_json);
            uint16_t crc = Modbus_CRC16((const unsigned char*)node_json, len);

            snprintf(mqtt_payload, sizeof(mqtt_payload),
                     "{\"CRC\":\"%04X\",\"DATA\":%s}", crc, node_json);

            // Kirim ke MQTT
            if (MQTT_Publish(mqtt_topic_pub, mqtt_payload, 1, 0) == MQTT_OK)
            {
                node_success_count++;
            }
            else
            {
                node_fail_count++;
            }

            // Delay antar publish
            HAL_Delay(100);
        }
        else
        {
            node_fail_count++;
        }

        // Delay antar request
        HAL_Delay(50);
    }

    // Selesai
    is_requesting = false;
}
//void MCU2_RegisCCO(void)
//{
//	char regcco[32];
//	snprintf(regcco, sizeof(regcco), "MAC_CCO");
//	MCU1_SendRequest(regcco,MCU1_TIMEOUT_MS);
//}

void MCU2_RegisAllNodes(void)
{
    // Set flag requesting
    is_requesting = true;
    node_success_count = 0;
    node_fail_count = 0;
    node_current_index = 0;

    // -----------------------------------------
    // Step 1: Request jumlah node
    // -----------------------------------------
    if (!MCU1_RequestNodeCount())
    {
        // Gagal dapat jumlah node
        is_requesting = false;
        snprintf(debug,sizeof(debug),"gagal counting");
//        MCU1_RequestNodeCount();

        return;
    }

    // Jika tidak ada node
    if (node_total_count == 0)
    {
        is_requesting = false;
        snprintf(debug,sizeof(debug),"dikirim 0");
        return;
    }

    SSD1309_ClearArea(80, 0, 120);
    snprintf(jml_node, sizeof(jml_node), "%d", node_total_count);
    SSD1309_ShowString(80,0,jml_node);
    // -----------------------------------------
    // Step 2: Request setiap node dan kirim ke MQTT
    // -----------------------------------------
    char node_json[300];
    char mqtt_payload[512];

    for (uint16_t i = 0; i < node_total_count; i++)
    {
        node_current_index = i;

        // Request data node
        if (MCU1_RegisNodeData(i, node_json, sizeof(node_json)))
        {
            // Buat payload MQTT dengan CRC
            // Format: {"CRC":"XXXX","DATA":{...node data...}}
            size_t len = strlen(node_json);
            uint16_t crc = Modbus_CRC16((const unsigned char*)node_json, len);

            snprintf(mqtt_payload, sizeof(mqtt_payload),
                     "{\"CRC\":\"%04X\",\"DATA\":%s}", crc, node_json);

            // Kirim ke MQTT
            if (MQTT_Publish(sta_topic_reg, mqtt_payload, 1, 0) == MQTT_OK)
            {
                node_success_count++;
            }
            else
            {
                node_fail_count++;
            }

            // Delay antar publish
            HAL_Delay(100);
        }
        else
        {
            node_fail_count++;
        }

        // Delay antar request
        HAL_Delay(50);
    }

    // Selesai
    is_requesting = false;
}

void TEST_SendReqCnt(void)
{
    test_send_count++;

    // Kirim "REQ:CNT\r\n" ke MCU1 via UART2
    char buf[32] = "REQ:CNT\n";

    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 1000);

    // Update debug
    snprintf(debug, sizeof(debug), "SENT #%lu: REQ:CNT", test_send_count);
}

// Parse nilai integer dari JSON
// Contoh: {"PWM":75} → return 75
int Parse_JSON_Int(const char *json, const char *key)
{
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":", key);

    char *ptr = strstr(json, search);
    if (ptr == NULL) return -9999;  // Key tidak ditemukan

    ptr += strlen(search);

    // Skip whitespace
    while (*ptr == ' ') ptr++;

    return atoi(ptr);
}

// Parse nilai string dari JSON
// Contoh: {"STA":"FFFF00000001"} → output = "FFFF00000001"
bool Parse_JSON_String(const char *json, const char *key, char *output, size_t output_size)
{
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":\"", key);

    char *ptr = strstr(json, search);
    if (ptr == NULL) return false;

    ptr += strlen(search);  // Sekarang ptr menunjuk ke awal string value

    // Copy sampai ketemu closing quote
    size_t i = 0;
    while (*ptr != '"' && *ptr != '\0' && i < output_size - 1)
    {
        output[i++] = *ptr++;
    }
    output[i] = '\0';

    return (i > 0);
}

// ============================================================================
//                    KIRIM COMMAND KE MCU1
// ============================================================================

// Kirim command Control Group ke MCU1
// Format: CMD:G,PWM:75\n
bool MCU2_SendControlGroup(int pwm_value)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "CMD:G,PWM:%d", pwm_value);

    snprintf(debug, sizeof(debug), "TX Group: PWM=%d", pwm_value);

    if (MCU1_SendRequest(cmd, MCU1_TIMEOUT_MS))
    {
        // Cek response OK dari MCU1
        if (strstr((const char*)mcu1_rx_buffer, "OK") != NULL)
        {
            snprintf(debug, sizeof(debug), "Group OK");
            return true;
        }
    }

    snprintf(debug, sizeof(debug), "Group FAIL");
    return false;
}

// Kirim command Control Individu ke MCU1
// Format: CMD:I,STA:FFFF00000001,PWM:50\n
bool MCU2_SendControlIndividu(const char *sta_mac, int pwm_value)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "CMD:I,STA:%s,PWM:%d", sta_mac, pwm_value);

    snprintf(debug, sizeof(debug), "TX Individu: %s PWM=%d", sta_mac, pwm_value);

    if (MCU1_SendRequest(cmd, MCU1_TIMEOUT_MS))
    {
        // Cek response OK dari MCU1
        if (strstr((const char*)mcu1_rx_buffer, "OK") != NULL)
        {
            snprintf(debug, sizeof(debug), "Individu OK");
            return true;
        }
    }

    snprintf(debug, sizeof(debug), "Individu FAIL");
    return false;
}

// ============================================================================
//                    COMMAND HANDLER (Update dari sebelumnya)
// ============================================================================

void Command_Handler(void)
{
    if (!mqtt_data_ready) return;

    // Jangan proses jika sedang dalam proses request
    if (is_requesting)
    {
        mqtt_data_ready = false;
        return;
    }

    // Ekstrak payload dari "DATA":{...}
    if (!Extract_Payload())
    {
        mqtt_data_ready = false;
        return;
    }
//     ✅ TAMBAHKAN DI SINI - tepat setelah blok if di atas

	if (strstr((const char*)payload_data, "+QMTSTAT:0,1") != NULL ||
		strstr((const char*)payload_data, "+QMTSTAT:0,2") != NULL ||
		strstr((const char*)payload_data, "+QMTSTAT:0,3") != NULL)
	{
		mqtt_disconnected = true;
	}


    // -----------------------------------------
    // Cek {"H":"R"} - Request semua data node
    // -----------------------------------------
    if (strstr((const char*)payload_data, "\"H\":\"R\"") != NULL)
    {
        // Mulai proses request dari MCU1
    	mqtt_data_ready = false;
    	MCU2_RegisAllNodes();
        MCU2_RequestAllNodes();
//    	TEST_SendReqCnt();

//	  char json_Y_payload[800];
//	  snprintf(json_Y_payload, sizeof(json_Y_payload), "{\"H\":\"D\",\"STA\":\"FFFF00000005\",\"V\":222.05,\"I\":20.2,\"PWM\":12,\"R\":0}");
//	  size_t lenY = strlen(json_Y_payload);
//	  uint16_t crcY = Modbus_CRC16((const unsigned char *)json_Y_payload, lenY);
//	  char json_Y_with_crc[1000];
//		  snprintf(json_Y_with_crc, sizeof(json_Y_with_crc),
//				  "{\"CRC\":\"%04X\",\"DATA\":%s}", crcY, json_Y_payload);
//	  MQTT_Publish(mqtt_topic_pub,json_Y_with_crc,1,0);

    }

    // -----------------------------------------
    // Tambahkan command lain di sini jika perlu
    // -----------------------------------------
    // else if (strstr((const char*)payload_data, "\"H\":\"W\"") != NULL)
    // {
    //     // Handle write command
    // }

    // Reset flag
    else if (strstr((const char*)payload_data, "\"H\":\"G\"") != NULL)
    {
        snprintf(debug, sizeof(debug), "CMD: Control Group");

        // Parse nilai PWM
        int pwm = Parse_JSON_Int((const char*)payload_data, "PWM");

        if (pwm >= 0 && pwm <= 100)
        {
            // Kirim command ke MCU1
            if (MCU2_SendControlGroup(pwm))
            {
                // Kirim response sukses ke MQTT
                char response[128];
                snprintf(response, sizeof(response),
                         "{\"H\":\"G\",\"PWM\":%d,\"STATUS\":\"OK\"}", pwm);

                size_t len = strlen(response);
                uint16_t crc = Modbus_CRC16((const unsigned char*)response, len);

                char mqtt_payload[256];
                snprintf(mqtt_payload, sizeof(mqtt_payload),
                         "{\"CRC\":\"%04X\",\"DATA\":%s}", crc, response);

                MQTT_Publish(mqtt_topic_pub, mqtt_payload, 1, 0);
            }
        }
        else
        {
            snprintf(debug, sizeof(debug), "PWM invalid: %d", pwm);
        }
        mqtt_data_ready = false;
    }

    // =========================================
    // 3. Cek {"H":"I"} - Control Individu
    // =========================================
    else if (strstr((const char*)payload_data, "\"H\":\"I\"") != NULL)
    {
        snprintf(debug, sizeof(debug), "CMD: Control Individu");

        // Parse MAC STA
        char sta_mac[16] = {0};
        if (!Parse_JSON_String((const char*)payload_data, "STA", sta_mac, sizeof(sta_mac)))
        {
            snprintf(debug, sizeof(debug), "STA not found");
            mqtt_data_ready = false;
            return;
        }

        // Parse nilai PWM
        int pwm = Parse_JSON_Int((const char*)payload_data, "PWM");

        if (pwm >= 0 && pwm <= 100)
        {
            // Kirim command ke MCU1
            if (MCU2_SendControlIndividu(sta_mac, pwm))
            {
                // Kirim response sukses ke MQTT
                char response[128];
                snprintf(response, sizeof(response),
                         "{\"H\":\"I\",\"STA\":\"%s\",\"PWM\":%d,\"STATUS\":\"OK\"}",
                         sta_mac, pwm);

                size_t len = strlen(response);
                uint16_t crc = Modbus_CRC16((const unsigned char*)response, len);

                char mqtt_payload[256];
                snprintf(mqtt_payload, sizeof(mqtt_payload),
                         "{\"CRC\":\"%04X\",\"DATA\":%s}", crc, response);

                MQTT_Publish(mqtt_topic_pub, mqtt_payload, 1, 0);
            }
        }
        else
        {
            snprintf(debug, sizeof(debug), "PWM invalid: %d", pwm);
        }
        mqtt_data_ready = false;
    }

    // Reset flag
    mqtt_data_ready = false;
}
