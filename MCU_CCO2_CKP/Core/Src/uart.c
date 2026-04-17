/*
 * uart.c
 * CARA A: ISR langsung isi bufferRx[] → dataMerge() → rcv_flag
 * Sama persis mekanisme referensi asli
 */

#include "uart.h"
#include "HLW8110_Access.h"   /* ← untuk akses rxtxVar, indx, length, dataMerge, reqADDR */
#include <string.h>
#include <stdio.h>

/* ============================================================
 * BUFFER DEFINITION
 * ============================================================ */
volatile char rxBuffer[RX_BUFFER_SIZE];
volatile char payload_data[PAY_LOAD_SIZE];
volatile uint16_t rx_index = 0;

volatile bool mqtt_data_ready = false;
volatile uint32_t mqtt_msg_count = 0;
volatile bool mqtt_disconnected = false;

static uint8_t rx_byte;

static const char QMTRECV_PATTERN[] = "+QMTRECV:";
static volatile uint8_t pattern_index = 0;

/* ============================================================
 * INITIALIZATION
 * ============================================================ */

void UART_Init_Buffer(void)
{
    ENTER_CRITICAL();
    memset((void*)rxBuffer, 0, RX_BUFFER_SIZE);
    rx_index = 0;
    mqtt_data_ready = false;
    mqtt_msg_count = 0;
    pattern_index = 0;
    EXIT_CRITICAL();

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/* ============================================================
 * BUFFER MANAGEMENT
 * ============================================================ */

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
    uint16_t last_index = 0;
    uint16_t current_index = 0;

    UART_ClearBuffer();

    while (HAL_GetTick() < absolute_deadline)
    {
        ENTER_CRITICAL();
        current_index = rx_index;
        EXIT_CRITICAL();

        if (current_index != last_index)
        {
            last_index = current_index;
            UART_ClearBuffer();
        }
        HAL_Delay(5);
    }
    UART_ClearBuffer();
}

/* ============================================================
 * UART RX INTERRUPT CALLBACK
 * ============================================================ */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* --------------------------------------------------------
     * USART1 - Komunikasi GSM/MQTT
     * -------------------------------------------------------- */
    if (huart->Instance == USART1)
    {
        if (rx_byte == QMTRECV_PATTERN[pattern_index])
        {
            if (pattern_index == 0)
            {
                rx_index = 0;
                rxBuffer[0] = '\0';
            }
            pattern_index++;

            if (pattern_index == 9)
            {
                rx_index = 0;
                for (uint8_t i = 0; i < 9; i++)
                    rxBuffer[i] = QMTRECV_PATTERN[i];
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
                rx_index = 0;
                rxBuffer[0] = '\0';
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
                mqtt_data_ready = true;

            if (rx_byte == '\n')
            {
                if (strstr((const char*)rxBuffer, "+QMTSTAT: 0,1") != NULL ||
                    strstr((const char*)rxBuffer, "+QMTSTAT: 0,2") != NULL ||
                    strstr((const char*)rxBuffer, "+QMTSTAT: 0,3") != NULL)
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

    /* --------------------------------------------------------
     * USART2 - Komunikasi MCU1 (CCO)
     * -------------------------------------------------------- */
    else if (huart->Instance == USART2)
    {
        MCU1_UART_RxCallback();
    }

    /* --------------------------------------------------------
     * USART3 - Komunikasi HLW8110/8112
     *
     * CARA A: Sama persis dengan referensi asli
     * ISR langsung isi rxtxVar.bufferRx[indx]
     * Saat indx == length → panggil dataMerge() → set rcv_flag
     * -------------------------------------------------------- */
    else if (huart->Instance == USART3)
    {
        /* Teruskan byte ke state machine parser */
        HLW_OnRxByte(rxtxVar.dataRx);

        /* Re-arm interrupt untuk byte berikutnya */
        HAL_UART_Receive_IT(&huart3, (uint8_t*)&rxtxVar.dataRx, 1);
    }
}

/* ============================================================
 * SEND FUNCTIONS
 * ============================================================ */

void UART_SendString(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str, (uint16_t)strlen(str), 5000);
}

void UART_SendATCommand(const char *cmd_no_crlf)
{
    UART_ClearBuffer();
    char buf[512];
    snprintf(buf, sizeof(buf), "%s\r\n", cmd_no_crlf);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)strlen(buf), 5000);
}

/* ============================================================
 * HELPER: Safe buffer search
 * ============================================================ */

static bool buffer_contains(const char *str)
{
    bool result = false;
    ENTER_CRITICAL();
    if (rx_index > 0)
        result = (strstr((const char*)rxBuffer, str) != NULL);
    EXIT_CRITICAL();
    return result;
}

/* ============================================================
 * WAIT FUNCTIONS
 * ============================================================ */

bool UART_WaitForOK(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (buffer_contains("OK"))    return true;
        if (buffer_contains("ERROR")) return false;
        HAL_Delay(5);
    }
    return false;
}

bool UART_WaitForURC(const char *urc_expected, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (buffer_contains(urc_expected)) return true;
        if (buffer_contains("ERROR"))      return false;
        HAL_Delay(5);
    }
    return false;
}

bool UART_WaitForPrompt(const char *prompt, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (buffer_contains(prompt)) return true;
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
        if (buffer_contains("OK"))    { ok_found = true; break; }
        if (buffer_contains("ERROR")) return false;
        HAL_Delay(5);
    }

    if (!ok_found) return false;

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_urc_ms)
    {
        if (buffer_contains(urc_expected)) return true;
        if (buffer_contains("ERROR"))      return false;
        HAL_Delay(5);
    }
    return false;
}

bool Extract_Payload(void)
{
    memset(payload_data, 0, sizeof(payload_data));

    char *start = strstr((const char*)rxBuffer, "\"DATA\":");
    if (start == NULL) return false;

    start += 7;
    while (*start == ' ') start++;
    if (*start != '{') return false;

    int bracket_count = 0;
    char *ptr = start;
    char *end = NULL;

    while (*ptr != '\0')
    {
        if (*ptr == '{')      bracket_count++;
        else if (*ptr == '}')
        {
            bracket_count--;
            if (bracket_count == 0) { end = ptr; break; }
        }
        ptr++;
    }

    if (end == NULL) return false;

    int len = (end - start) + 1;
    if (len >= (int)sizeof(payload_data))
        len = sizeof(payload_data) - 1;

    strncpy(payload_data, start, len);
    payload_data[len] = '\0';
    return true;
}
