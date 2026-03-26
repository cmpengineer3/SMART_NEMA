/*
 * com_mcu2.c
 *
 *  Created on: Jan 27, 2026
 *      Author: firza
 */


/*
 * mcu1_mcu2_comm.c
 *
 * Tambahan code untuk MCU1 (CCO) - Handle request dari MCU2
 * Tambahkan ke uart_cco.c atau buat file terpisah
 *
 * Format Komunikasi:
 * - MCU2 kirim "REQ:CNT\n"  → MCU1 reply "CNT:5\n" (jumlah node)
 * - MCU2 kirim "REQ:0\n"    → MCU1 reply JSON data node index 0
 * - MCU2 kirim "REQ:1\n"    → MCU1 reply JSON data node index 1
 * - dst...
 */

#include "uart_cco.h"
#include "com_mcu2.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
// ============================================================================
//                    BUFFER UNTUK KOMUNIKASI MCU2
// ============================================================================

#define MCU2_RX_BUFFER_SIZE  64

static char mcu2_rx_buffer[MCU2_RX_BUFFER_SIZE];
static volatile uint16_t mcu2_rx_index = 0;
static uint8_t mcu2_rx_byte;

// Flag request dari MCU2
volatile bool mcu2_request_ready = false;

// ============================================================================
//                    INIT UART2 INTERRUPT (untuk terima dari MCU2)
// ============================================================================

void MCU2_Comm_Init(void)
{
    memset(mcu2_rx_buffer, 0, MCU2_RX_BUFFER_SIZE);
    mcu2_rx_index = 0;
    mcu2_request_ready = false;

    // Start interrupt untuk UART2 (komunikasi dengan MCU2)
    HAL_UART_Receive_IT(&huart2, &mcu2_rx_byte, 1);
}

// ============================================================================
//                    CALLBACK UART2 (Terima dari MCU2)
// ============================================================================
// CATATAN: Tambahkan ini ke HAL_UART_RxCpltCallback yang sudah ada
// atau modifikasi callback yang existing

void MCU2_UART_RxCallback(void)
{
    // Simpan byte
    if (mcu2_rx_index < MCU2_RX_BUFFER_SIZE - 1)
    {
        mcu2_rx_buffer[mcu2_rx_index++] = mcu2_rx_byte;
        //mcu2_rx_index++;
        mcu2_rx_buffer[mcu2_rx_index] = '\0';

        // Deteksi akhir pesan (newline)
        if (mcu2_rx_byte == '\n')
        {
            mcu2_request_ready = true;
        }
    }
    else
    {
        // Buffer overflow - reset
        mcu2_rx_index = 0;
        mcu2_rx_buffer[0] = '\0';
    }

    // Re-enable interrupt
    HAL_UART_Receive_IT(&huart2, &mcu2_rx_byte, 1);
}

// ============================================================================
//                    KIRIM DATA NODE KE MCU2
// ============================================================================

// Kirim jumlah node
void MCU1_SendNodeCount(void)
{
    char buffer[32];
    int len = snprintf(buffer, sizeof(buffer), "CNT:%u\n", mac_sta_count);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000);
}

// Kirim data 1 node berdasarkan index
void MCU1_SendNodeByIndex(uint16_t index)
{
    char buffer[256];

    // Cek index valid
    if (index >= mac_sta_count)
    {
        // Kirim error/empty response
        int len = snprintf(buffer, sizeof(buffer), "ERR:INVALID_INDEX\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000);
        return;
    }

    STA_Node* node = &mac_sta_list[index];

    float voltage = (float)node->v / 10.0f;
    float current = (float)node->i / 100.0f;

    int len = snprintf(buffer, sizeof(buffer),
             "{\"H\":\"D\",\"IDX\":%u,\"STA\":\"%s\",\"La\":%g,\"Lo\":%g,"
             "\"V\":%.1f,\"I\":%.2f,\"P\":%u,"
             "\"R\":%u,\"PWM\":%u,\"COND\":%d}\n",
             index,
             node->mac_sta,
             node->La,
             node->Lo,
             voltage,
             current,
             node->p,
             node->relay,
             node->dim,
             node->cond ? 1 : 0);

    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000);
}

void MCU1_SendRegByIndex(uint16_t index)
{
    char buffer[256];


    // Cek index valid
    if (index >= mac_sta_count)
    {
        // Kirim error/empty response
        int len = snprintf(buffer, sizeof(buffer), "ERR:INVALID_INDEX\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000);
        return;
    }

    STA_Node* node = &mac_sta_list[index];

    int len = snprintf(buffer, sizeof(buffer),
             "{\"H\":\"RS\",\"IDX\":%u,\"STA\":\"%s\"}\n",
             index,
             node->mac_sta);

    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000);
}

void MCU1_SendRegCCO(void)
{
    char buffer[256];
    int len = snprintf(buffer, sizeof(buffer),
             "{\"H\":\"RC\",\"CCO\":\"%s\"}\n",
             mac_cco);

    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000);
}

// Kirim PWM ke semua STA (Group Control)
bool MCU1_SendPWM_Group(int pwm_value)
{
    bool all_success = true;

    for (uint16_t i = 0; i < mac_sta_count; i++)
    {
        // Format payload untuk STA
        char payload[128];
        snprintf(payload, sizeof(payload),
                 "\"DATA\":{\"UID\":\"%s\",\"HEADER\":\"G\",\"PWM\":\"%d\"}",
                 mac_cco, pwm_value);

        // Kirim ke STA
        UART_CCO_ClearRecvBuffer();

        if (CCO_SendData_SENDEX(mac_sta_list[i].mac_sta, payload))
        {
            char response[256];
            if (CCO_WaitAndParse_RECV(mac_sta_list[i].mac_sta, response, sizeof(response), POLLING_TIMEOUT_MS))
            {
                // Update dim value di list
                mac_sta_list[i].dim = (uint8_t)pwm_value;
            }
            else
            {
                all_success = false;
            }
        }
        else
        {
            all_success = false;
        }

        HAL_Delay(100);  // Delay antar STA
    }

    return all_success;
}

// Kirim PWM ke STA tertentu (Individual Control)
bool MCU1_SendPWM_Individu(const char *sta_mac, int pwm_value)
{
    // Cari index STA
    uint16_t index = 0;
    bool found = CCO_FindNodeByMAC(sta_mac, &index);

    if (!found)
    {
        return false;  // STA tidak ditemukan
    }

    // Format payload untuk STA
    char payload[128];
    snprintf(payload, sizeof(payload),
             "\"DATA\":{\"UID\":\"%s\",\"HEADER\":\"G\",\"PWM\":\"%d\"}",
             mac_cco, pwm_value);

    // Kirim ke STA
    UART_CCO_ClearRecvBuffer();

    if (CCO_SendData_SENDEX(sta_mac, payload))
    {
        char response[256];
        if (CCO_WaitAndParse_RECV(sta_mac, response, sizeof(response), POLLING_TIMEOUT_MS))
        {
            // Update dim value di list
            mac_sta_list[index].dim = (uint8_t)pwm_value;
            return true;
        }
    }

    return false;
}

// ============================================================================
//                    REPLY KE MCU2
// ============================================================================

void MCU1_SendOK(void)
{
    char buffer[16] = "OK\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 1000);
}

void MCU1_SendError(const char *msg)
{
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "ERR:%s\n", msg);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000);
}
// ============================================================================
//                    PROSES REQUEST DARI MCU2
// ============================================================================

void MCU1_ProcessMCU2Request(void)
{
    if (!mcu2_request_ready) return;

    // ------------------------------------------
    // Parse request
    // Format: "REQ:CNT\n" atau "REQ:0\n"
    // ------------------------------------------

    // Cek request jumlah node
    if (strstr(mcu2_rx_buffer, "REQ:CNT") != NULL)
    {
    	Save_Node();
        HAL_Delay(500);
//          Send_GPIO_Toggle();
        // 2. Polling semua STA dengan AT+SENDEX
        CCO_PollAllSTA_SENDEX();
        HAL_Delay(500);
        MCU1_SendNodeCount();
    }
    // Cek request node by index
    else if (strncmp(mcu2_rx_buffer, "REQ:", 4) == 0)
    {
        // Parse index dari "REQ:X"
        int index = atoi(mcu2_rx_buffer + 4);

        if (index >= 0)
        {
            MCU1_SendNodeByIndex((uint16_t)index);
        }
    }
    else if (strncmp(mcu2_rx_buffer, "MAC_CCO", 4) == 0)
    {
        // Parse index dari "REQ:X"
        int index = atoi(mcu2_rx_buffer + 4);

        if (index >= 0)
        {
        	MCU1_SendRegCCO();
        }
    }
    else if (strncmp(mcu2_rx_buffer, "REG:", 4) == 0)
    {
        // Parse index dari "REQ:X"
        int index = atoi(mcu2_rx_buffer + 4);

        if (index >= 0)
        {
        	MCU1_SendRegByIndex((uint16_t)index);
        }
    }
    // 3. CMD:G,PWM:XX - Control Group
    // ------------------------------------------
    else if (strstr(mcu2_rx_buffer, "CMD:G") != NULL)
    {
        // Parse PWM value
        char *pwm_ptr = strstr(mcu2_rx_buffer, "PWM:");
        if (pwm_ptr != NULL)
        {
            int pwm_value = atoi(pwm_ptr + 4);

            if (pwm_value >= 0 && pwm_value <= 100)
            {
                // Kirim ke semua STA
                if (MCU1_SendPWM_Group(pwm_value))
                {
                    MCU1_SendOK();
                }
                else
                {
                    MCU1_SendError("PARTIAL");  // Sebagian gagal
                }
            }
            else
            {
                MCU1_SendError("PWM_RANGE");
            }
        }
        else
        {
            MCU1_SendError("NO_PWM");
        }
    }
    // ------------------------------------------
    // 4. CMD:I,STA:XXXX,PWM:XX - Control Individu
    // ------------------------------------------
    else if (strstr(mcu2_rx_buffer, "CMD:I") != NULL)
    {
        // Parse STA MAC
        char *sta_ptr = strstr(mcu2_rx_buffer, "STA:");
        char *pwm_ptr = strstr(mcu2_rx_buffer, "PWM:");

        if (sta_ptr != NULL && pwm_ptr != NULL)
        {
            // Extract MAC (12 karakter setelah "STA:")
            char sta_mac[13] = {0};
            strncpy(sta_mac, sta_ptr + 4, 12);
            sta_mac[12] = '\0';

            // Extract PWM
            int pwm_value = atoi(pwm_ptr + 4);

            if (pwm_value >= 0 && pwm_value <= 100)
            {
                // Kirim ke STA tertentu
                if (MCU1_SendPWM_Individu(sta_mac, pwm_value))
                {
                    MCU1_SendOK();
                }
                else
                {
                    MCU1_SendError("STA_FAIL");
                }
            }
            else
            {
                MCU1_SendError("PWM_RANGE");
            }
        }
        else
        {
            MCU1_SendError("INVALID_CMD");
        }
    }

    // Clear buffer dan reset flag
    memset(mcu2_rx_buffer, 0, MCU2_RX_BUFFER_SIZE);
    mcu2_rx_index = 0;
    mcu2_request_ready = false;
}

// ============================================================================
//                    UPDATE HAL_UART_RxCpltCallback
// ============================================================================
/*
 * CATATAN: Modifikasi HAL_UART_RxCpltCallback yang sudah ada di uart_cco.c
 * Tambahkan handling untuk USART2:
 *
 * void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 * {
 *     if (huart->Instance == USART1)
 *     {
 *         // ... kode existing untuk PLC ...
 *     }
 *     else if (huart->Instance == USART2)
 *     {
 *         // Handle request dari MCU2
 *         MCU2_UART_RxCallback();
 *     }
 * }
 */
