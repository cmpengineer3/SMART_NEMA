/*
 * uart.c (SIMPLIFIED VERSION)
 *
 * Fitur:
 * 1. Semua data serial masuk ke rxBuffer
 * 2. Saat "+QMTRECV:" terdeteksi → CLEAR buffer dulu → baru simpan data baru
 * 3. Data mentah tanpa parsing
 * 4. Bisa monitor langsung di Live Expression: rxBuffer
 */

#include "uart.h"
#include <string.h>
#include <stdio.h>

/* ============== BUFFER DEFINITION ============== */
volatile char rxBuffer[RX_BUFFER_SIZE];
volatile char payload_data[PAY_LOAD_SIZE];
volatile uint16_t rx_index = 0;

// Flag dan counter untuk monitoring
volatile bool mqtt_data_ready = false;
volatile uint32_t mqtt_msg_count = 0;
volatile bool mqtt_disconnected = false;

// Byte untuk interrupt
static uint8_t rx_byte;

// Pattern matching untuk deteksi +QMTRECV:
static const char QMTRECV_PATTERN[] = "+QMTRECV:";
static volatile uint8_t pattern_index = 0;
volatile uint8_t hlw_rx_byte;

/* ============== INITIALIZATION ============== */

void UART_Init_Buffer(void)
{
    ENTER_CRITICAL();

    memset((void*)rxBuffer, 0, RX_BUFFER_SIZE);
    rx_index = 0;
    mqtt_data_ready = false;
    mqtt_msg_count = 0;
    pattern_index = 0;

    EXIT_CRITICAL();

    // Start UART interrupt
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/* ============== BUFFER MANAGEMENT ============== */

void UART_ClearBuffer(void)
{
    ENTER_CRITICAL();
    memset((void*)rxBuffer, 0, RX_BUFFER_SIZE);
    rx_index = 0;
    EXIT_CRITICAL();
}

void UART_FlushRx(uint32_t wait_ms)
{
    uint32_t absolute_deadline = HAL_GetTick() + wait_ms;
    uint32_t stable_start = HAL_GetTick();
    uint16_t last_index = 0;
    uint16_t current_index = 0;

    UART_ClearBuffer();

    while (HAL_GetTick() < absolute_deadline)  // ← batas ABSOLUT, tidak bisa diperpanjang
    {
        ENTER_CRITICAL();
        current_index = rx_index;
        EXIT_CRITICAL();

        if (current_index != last_index)
        {
            last_index = current_index;
            stable_start = HAL_GetTick();
            UART_ClearBuffer();
        }
        HAL_Delay(5);
    }
    UART_ClearBuffer();
}

/* ============== UART RX INTERRUPT ============== */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // ----------------------------------------
    // USART1 - Komunikasi dengan GSM Module (existing)
    // ----------------------------------------
    if (huart->Instance == USART1)
    {
        // ... kode existing untuk +QMTRECV ...
        // (sama seperti sebelumnya)

        // Cek apakah byte cocok dengan pattern
        if (rx_byte == QMTRECV_PATTERN[pattern_index])
        {
            pattern_index++;

            if (pattern_index == 9)
            {
//                memset((void*)rxBuffer, 0, RX_BUFFER_SIZE);
                rx_index = 0;

                for (uint8_t i = 0; i < 9; i++)
                {
                    rxBuffer[i] = QMTRECV_PATTERN[i];
                }
                rx_index = 9;
                rxBuffer[rx_index] = '\0';

                pattern_index = 0;
                mqtt_msg_count++;
                mqtt_data_ready = false;

                HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
                return;
            }
        }
        else
        {
            if (rx_byte == '+')
            {
                pattern_index = 1;
            }
            else
            {
                pattern_index = 0;
            }
        }

        if (rx_index < RX_BUFFER_SIZE - 1)
        {
            rxBuffer[rx_index] = rx_byte;
            rx_index++;
            rxBuffer[rx_index] = '\0';

            if (rx_byte == '\n' && rxBuffer[0] == '+' && rxBuffer[1] == 'Q')
            {
                mqtt_data_ready = true;
            }
            // ✅ TAMBAHKAN DI SINI - tepat setelah blok if di atas
            if (rx_byte == '\n')
            {
                if (strstr((const char*)rxBuffer, "+QMTSTAT:0,1") != NULL ||
                    strstr((const char*)rxBuffer, "+QMTSTAT:0,2") != NULL ||
                    strstr((const char*)rxBuffer, "+QMTSTAT:0,3") != NULL)
                {
                    mqtt_disconnected = true;
                }
            }
        }
        else
        {
            rx_index = 0;
            rxBuffer[0] = '\0';
        }

        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
    // ----------------------------------------
    // USART3 - Komunikasi dengan MCU1 (TAMBAH INI)
    // ----------------------------------------
    else if (huart->Instance == USART2)
    {
        MCU1_UART_RxCallback();
    }
//    else if (huart->Instance == USART3)  // HLW8110
//    {
//        HLW_OnRxByte(hlw_rx_byte);
//        HAL_UART_Receive_IT(&huart3, &hlw_rx_byte, 1);
//    }
}

/* ============== SEND FUNCTIONS ============== */

void UART_SendString(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str, (uint16_t)strlen(str), 5000);
}

void UART_SendATCommand(const char *cmd_no_crlf)
{
    // JANGAN clear buffer di sini jika ingin melihat +QMTRECV
    // Hanya clear saat kirim AT command yang butuh response
    UART_ClearBuffer();

    char buf[512];
    snprintf(buf, sizeof(buf), "%s\r\n", cmd_no_crlf);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)strlen(buf), 5000);
}

/* ============== HELPER: Safe buffer search ============== */

static bool buffer_contains(const char *str)
{
    bool result = false;
    ENTER_CRITICAL();
    if (rx_index > 0)
    {
        result = (strstr((const char*)rxBuffer, str) != NULL);
    }
    EXIT_CRITICAL();
    return result;
}

/* ============== WAIT FUNCTIONS ============== */

bool UART_WaitForOK(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (buffer_contains("OK"))
        {
            return true;
        }
        if (buffer_contains("ERROR"))
        {
            return false;
        }
        HAL_Delay(5);
    }
    return false;
}

bool UART_WaitForURC(const char *urc_expected, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (buffer_contains(urc_expected))
        {
            return true;
        }
        if (buffer_contains("ERROR"))
        {
            return false;
        }
        HAL_Delay(5);
    }
    return false;
}

bool UART_WaitForPrompt(const char *prompt, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (buffer_contains(prompt))
        {
            return true;
        }
        HAL_Delay(5);
    }
    return false;
}

bool UART_WaitFor_OK_Then_URC(const char *urc_expected,
                              uint32_t timeout_ok_ms,
                              uint32_t timeout_urc_ms)
{
    uint32_t start = HAL_GetTick();
    bool ok_found = false;

    while ((HAL_GetTick() - start) < timeout_ok_ms)
    {
        if (buffer_contains("OK"))
        {
            ok_found = true;
            break;
        }
        if (buffer_contains("ERROR"))
        {
            return false;
        }
        HAL_Delay(5);
    }

    if (!ok_found)
    {
        return false;
    }

    start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_urc_ms)
    {
        if (buffer_contains(urc_expected))
        {
            return true;
        }
        if (buffer_contains("ERROR"))
        {
            return false;
        }
        HAL_Delay(5);
    }

    return false;
}

bool Extract_Payload(void)
{
    // Clear payload buffer
    memset(payload_data, 0, sizeof(payload_data));

    // Cari posisi "DATA":
    char *start = strstr((const char*)rxBuffer, "\"DATA\":");
    if (start == NULL)
    {
        return false;  // Tidak ada "DATA"
    }

    // Skip "DATA": (7 karakter)
    start += 7;

    // Skip whitespace jika ada
    while (*start == ' ') start++;

    // Sekarang start menunjuk ke { dari payload
    if (*start != '{')
    {
        return false;  // Format salah
    }

    // Hitung bracket untuk menemukan akhir payload
    // Karena payload bisa nested: {"A":{"B":1}}
    int bracket_count = 0;
    char *ptr = start;
    char *end = NULL;

    while (*ptr != '\0')
    {
        if (*ptr == '{')
        {
            bracket_count++;
        }
        else if (*ptr == '}')
        {
            bracket_count--;
            if (bracket_count == 0)
            {
                end = ptr;
                break;
            }
        }
        ptr++;
    }

    if (end == NULL)
    {
        return false;  // Tidak menemukan closing bracket
    }

    // Copy payload (termasuk { dan })
    int len = (end - start) + 1;
    if (len >= sizeof(payload_data))
    {
        len = sizeof(payload_data) - 1;
    }

    strncpy(payload_data, start, len);
    payload_data[len] = '\0';

    return true;
}

//void Command_Handler(void)
//{
//    if (!mqtt_data_ready) return;
//
//    // Ekstrak payload dari "DATA":{...}
//    if (!Extract_Payload())
//    {
//        mqtt_data_ready = false;
//        return;
//    }
//
//    // Sekarang proses payload_data
//    // payload_data = {"H":"R","CMD":"ON","PWM":75}
//
//    // -----------------------------------------
//    // Cara 1: Cek kata kunci langsung
//    // -----------------------------------------
//    if (strstr(payload_data, "\"H\":\"R\"") != NULL)
//    {
//        // Handle request
//	  char json_Y_payload[800];
//	  snprintf(json_Y_payload, sizeof(json_Y_payload), "{\"H\":\"D\",\"STA\":\"FFFF00000005\",\"V\":222.05,\"I\":20.2,\"PWM\":12,\"R\":0}");
//	  size_t lenY = strlen(json_Y_payload);
//	  uint16_t crcY = Modbus_CRC16((const unsigned char *)json_Y_payload, lenY);
//	  char json_Y_with_crc[1000];
//		  snprintf(json_Y_with_crc, sizeof(json_Y_with_crc),
//				  "{\"CRC\":\"%04X\",\"DATA\":%s}", crcY, json_Y_payload);
//	  MQTT_Publish(mqtt_topic_pub,json_Y_with_crc,1,0);
//	  HAL_Delay(500);
//    }
//}
