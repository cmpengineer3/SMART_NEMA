/*
 * function.c
 *
 *  Created on: Dec 31, 2025
 *      Author: firza
 *      Modified: Optimized GPS handling with priority and auto-stop
 */
#include "function.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "math.h"
#include <ctype.h>

// ==================== GLOBAL VARIABLES ====================
uint32_t send_interval = 20000;
uint32_t lastSendTime = 0;

char rxBuffer[RX_BUFFER_STA];
char recvBuffer[RECV_BUFFER_SIZE];
char txBuffer[TX_BUFFER_STA];

char mac_sta[13];
char mac_cco[13];

char uart_tx_last[128] = {0};
uint16_t uart_tx_length = 0;

volatile bool recv_data_ready = false;
volatile uint8_t rx_byte;
volatile uint16_t rx_index = 0;
volatile uint32_t last_rx_tick = 0;

volatile bool rx_complete = false;
volatile bool data_ready_to_process = false;

// ==================== TEMPORARY BUFFER FOR JSON PARSING ====================
static char temp_value[32];

uint32_t delays = 5000;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef htim2;

// ==================== HLW CONFIG ====================
uint32_t DataRCV_HLW;
int indx_HLW;
uint8_t buffer_HLW[25];

#define koefisienVoltage 3.006
#define koefisienCurrent 0.31
#define koefisienPower 0.909

Param calc;
Param *ptr = &calc;

dataMerge merge;
dataMerge *addr = &merge;

int pulse = 15999;
int dimming = 0;
int dutytest = 100, relaytest = 0;

// ==================== GPS CONFIG ====================
#define number_of_variable  50
#define string_length_size  15

char time_local[10];
char new_string[number_of_variable][string_length_size];
char gga_string[number_of_variable][string_length_size];

volatile uint8_t rxData;                    // Volatile untuk ISR
char gpsBuffer[256];                        // Dikurangi dari 512, GPGGA max ~82 chars
volatile uint16_t gpsIndex = 0;
int hh, mm, ss;

float sendLat = 0.0f;
float sendLng = 0.0f;

// ===== GPS STATE MANAGEMENT =====
volatile bool gps_sentence_ready = false;   // Flag: ada sentence siap diproses
volatile bool gps_enabled = true;           // Flag: GPS masih aktif
bool gps_fix_obtained = false;              // Flag: sudah dapat fix valid
uint8_t gps_fix_quality = 0;                // 0=invalid, 1=GPS fix, 2=DGPS fix

// ==================== UART1 (STA) FUNCTIONS ====================

void UART1_StartUart(void)
{
    memset(mac_sta, 0, sizeof(mac_sta));
    memset(mac_cco, 0, sizeof(mac_cco));
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_byte, 1);
}

void UART_STA_ClearBuffer(void)
{
    rx_index = 0;
    memset(rxBuffer, 0, RX_BUFFER_STA);
}

/**
 * @brief UART RX Complete Callback - OPTIMIZED
 *        UART1 dan UART2 dihandle dengan minimal processing
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // ===== UART1 (STA Communication) - HIGH PRIORITY =====
        uint32_t current_tick = HAL_GetTick();

        // Auto clear jika jeda > 100ms
        if ((current_tick - last_rx_tick) > 100 && rx_index > 0)
        {
            rx_index = 0;
            memset(rxBuffer, 0, RX_BUFFER_STA);
        }
        last_rx_tick = current_tick;

        // Simpan byte
        if (rx_index < RX_BUFFER_STA - 1)
        {
            rxBuffer[rx_index++] = rx_byte;
            rxBuffer[rx_index] = '\0';

            // Detect +RECV complete (diakhiri newline)
            if (rx_byte == '\n' && strstr(rxBuffer, "+RECV:") != NULL)
            {
                if (!recv_data_ready)
                {
                    strcpy(recvBuffer, rxBuffer);
                    recv_data_ready = true;
                }
                rx_index = 0;
                memset(rxBuffer, 0, RX_BUFFER_STA);
            }
        }
        else
        {
            rx_index = 0;
            memset(rxBuffer, 0, RX_BUFFER_STA);
        }

        HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_byte, 1);
    }
    else if (huart->Instance == USART2)
    {
        // ===== UART2 (GPS) - LOW PRIORITY, MINIMAL PROCESSING =====
        // Hanya proses jika GPS masih enabled
        if (gps_enabled)
        {
            if (gpsIndex < sizeof(gpsBuffer) - 1)
            {
                gpsBuffer[gpsIndex++] = rxData;

                // Set flag hanya saat dapat newline (akhir NMEA sentence)
                if (rxData == '\n')
                {
                    gpsBuffer[gpsIndex] = '\0';
                    gps_sentence_ready = true;  // Flag untuk main loop
                    gpsIndex = 0;
                }
            }
            else
            {
                // Buffer overflow protection
                gpsIndex = 0;
            }

            HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxData, 1);
        }
        // Jika gps_enabled = false, TIDAK restart interrupt = GPS berhenti
    }
}

void UART_TX_ClearBuffer(void)
{
    memset(txBuffer, 0, TX_BUFFER_STA);
}

void UART_STA_SendRaw(const char *s)
{
    strncpy(uart_tx_last, s, sizeof(uart_tx_last) - 1);
    uart_tx_last[sizeof(uart_tx_last) - 1] = '\0';
    uart_tx_length = strlen(s);

    HAL_UART_Transmit(&huart1, (uint8_t*)s, uart_tx_length, 1000);
}

void UART_STA_SendAT(const char *cmd)
{
    UART_STA_ClearBuffer();
    char t[128];
    snprintf(t, sizeof(t), "%s\r\n", cmd);
    UART_STA_SendRaw(t);
}

bool UART_STA_WaitFor(const char *keyword, uint32_t timeout)
{
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout)
    {
        if (strstr(rxBuffer, keyword) != NULL)
            return true;
        HAL_Delay(1);
    }
    return false;
}

bool UART_STA_SendCmdWait(const char *cmd, const char *urc, uint32_t t_ok, uint32_t t_urc)
{
    UART_STA_ClearBuffer();
    UART_STA_SendAT(cmd);

    if (!UART_STA_WaitFor("OK", t_ok))
        return false;

    if (urc == NULL) return true;

    if (!UART_STA_WaitFor(urc, t_urc))
        return false;

    // Parse MAC jika urc adalah +MAC
    if (strstr(urc, "+MAC") != NULL)
    {
        char *p = strstr(rxBuffer, "+MAC:");
        if (p)
            strncpy(mac_sta, p + 5, sizeof(mac_sta) - 1);
    }
    return true;
}

char* ExtractJsonValue(const char *json, const char *key)
{
    if (json == NULL || key == NULL)
        return NULL;

    char search[32];
    snprintf(search, sizeof(search), "\"%s\"", key);

    char *pos = strstr(json, search);
    if (pos == NULL)
        return NULL;

    pos = strchr(pos, ':');
    if (pos == NULL)
        return NULL;
    pos++;

    while (*pos == ' ' || *pos == '\t') pos++;

    if (*pos == '"')
    {
        pos++;
        int i = 0;
        while (*pos != '"' && *pos != '\0' && i < 31)
        {
            temp_value[i++] = *pos++;
        }
        temp_value[i] = '\0';
    }
    else
    {
        int i = 0;
        while (*pos != ',' && *pos != '}' && *pos != '\0' && i < 31)
        {
            temp_value[i++] = *pos++;
        }
        temp_value[i] = '\0';
    }

    return temp_value;
}

// ============================================================================
//                        RECV PROCESSING
// ============================================================================

bool ParsePayload(const char *payload, ParsedCommand_t *cmd)
{
    memset(cmd, 0, sizeof(ParsedCommand_t));
    cmd->header = HEADER_UNKNOWN;
    cmd->pwm_value = -1;
    cmd->valid = false;

    if (payload == NULL)
        return false;

    char *data_start = strstr(payload, "\"DATA\":");
    if (data_start == NULL)
        data_start = strstr(payload, "DATA\":");

    if (data_start == NULL)
        return false;

    char *header = ExtractJsonValue(data_start, "HEADER");
    if (header == NULL)
        return false;

    if (strcmp(header, "RQ") == 0)
    {
        cmd->header = HEADER_RQ;
    }
    else if (strcmp(header, "G") == 0)
    {
        cmd->header = HEADER_G;

        char *pwm = ExtractJsonValue(data_start, "PWM");
        if (pwm != NULL)
        {
            cmd->pwm_value = atoi(pwm);
            if (cmd->pwm_value < 0) cmd->pwm_value = 0;
            if (cmd->pwm_value > 100) cmd->pwm_value = 100;
        }
    }
    else if (strcmp(header, "S") == 0)
    {
        cmd->header = HEADER_S;

        char *sta = ExtractJsonValue(data_start, "STA");
        if (sta != NULL)
        {
            strncpy(cmd->sta_mac, sta, sizeof(cmd->sta_mac) - 1);
        }

        char *pwm = ExtractJsonValue(data_start, "PWM");
        if (pwm != NULL)
        {
            cmd->pwm_value = atoi(pwm);
            if (cmd->pwm_value < 0) cmd->pwm_value = 0;
            if (cmd->pwm_value > 100) cmd->pwm_value = 100;
        }
    }
    else
    {
        return false;
    }

    cmd->valid = true;
    return true;
}

void ProcessRecv(void)
{
    if (!recv_data_ready)
        return;

    char *p = strstr(recvBuffer, "+RECV:");
    if (p == NULL)
        goto cleanup;

    p += 6;

    char *comma1 = strchr(p, ',');
    if (comma1 == NULL) goto cleanup;

    char *comma2 = strchr(comma1 + 1, ',');
    if (comma2 == NULL) goto cleanup;

    char *payload = comma2 + 1;

    ParsedCommand_t cmd;
    if (!ParsePayload(payload, &cmd))
    {
        blinkLED(LED_IND_Port, LED_IND_Pin, 1, 50);
        goto cleanup;
    }

    switch (cmd.header)
    {
        case HEADER_RQ:
            Handle_RQ();
            break;

        case HEADER_G:
            Handle_G(cmd.pwm_value);
            break;

        case HEADER_S:
            Handle_S(cmd.sta_mac, cmd.pwm_value);
            break;

        default:
            blinkLED(LED_IND_Port, LED_IND_Pin, 2, 50);
            break;
    }

cleanup:
    memset(recvBuffer, 0, sizeof(recvBuffer));
    recv_data_ready = false;
}

// ============================================================================
//                        COMMAND HANDLERS
// ============================================================================

void Handle_RQ(void)
{
    blinkLED(LED3_MAC_Port, LED3_MAC_Pin, 1, 30);
    HLW_ReadData();
    SendDataResponse();
}

void Handle_G(int pwm_value)
{
    blinkLED(LED3_MAC_Port, LED3_MAC_Pin, 2, 30);
    dutytest = pwm_value;
    Set_PWM1_Duty((uint8_t)pwm_value);
    HLW_ReadData();
    SendDataResponse();
}

void Handle_S(const char *target_mac, int pwm_value)
{
    if (strcmp(target_mac, mac_sta) != 0)
    {
        return;
    }

    blinkLED(LED3_MAC_Port, LED3_MAC_Pin, 3, 30);
    dutytest = pwm_value;
    Set_PWM1_Duty((uint8_t)pwm_value);
    HLW_ReadData();
    SendDataResponse();
}

// ============================================================================
//                        SEND RESPONSE
// ============================================================================

void SendDataResponse(void)
{
    char response[128];

    snprintf(response, sizeof(response),
             "\"DATA\":{\"UID\":\"%s\",\"STA\":\"%s\",\"V\":\"%.1f\",\"I\":\"%.2f\",\"P\":\"%.1f\",\"PWM\":\"%d\"}",
             mac_cco,
             mac_sta,
             ptr->Voltage,
             ptr->Current,
             ptr->activePower,
             pulse);

    uint16_t len = strlen(response);
    strncpy(txBuffer, response, sizeof(txBuffer) - 1);
    txBuffer[sizeof(txBuffer) - 1] = '\0';

    UART_STA_SendRaw("+++");
    HAL_Delay(200);

    UART_STA_ClearBuffer();

    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+SENDEX=%s,%d\r\n", mac_cco, len);
    UART_STA_SendRaw(cmd);

    HAL_Delay(100);

    UART_STA_SendRaw(response);

    UART_STA_WaitFor("OK", 3000);
}

// ==================== MAC FUNCTIONS ====================

bool Save_Mac(void)
{
    int retry = 0;
    const int max_retry_mac = 3;

    while (retry < max_retry_mac)
    {
        retry++;

        UART_STA_SendRaw("+++");
        HAL_Delay(200);

        if (UART_STA_SendCmdWait("AT+MAC?", "+MAC:", 1000, 2000))
        {
            blinkLED(LED3_MAC_Port, LED3_MAC_Pin, 2, 50);
            return true;
        }
        else
        {
            blinkLED(LED_IND_Port, LED_IND_Pin, 1, 50);
        }
    }
    return false;
}

bool Get_CCO_Mac(void)
{
    int retry = 0;
    const int max_retry_mac = 3;

    while (retry < max_retry_mac)
    {
        retry++;
        UART_STA_SendRaw("+++");
        HAL_Delay(200);
        if (UART_STA_SendCmdWait("AT+CCOMAC?", "+CCOMAC:", 1000, 2000))
        {
            char *p = strstr(rxBuffer, "+CCOMAC:");
            if (p)
            {
                p += 8;
                strncpy(mac_cco, p, 12);
                mac_cco[12] = '\0';

                for (int i = 11; i >= 0; i--)
                {
                    if (mac_cco[i] == '\r' || mac_cco[i] == '\n' || mac_cco[i] == ' ')
                        mac_cco[i] = '\0';
                    else
                        break;
                }
                return true;
            }
        }
    }
    return false;
}

// ============================================================================
//                        HLW8012 FUNCTIONS
// ============================================================================

void HLW_Init(void)
{
    indx_HLW = 0;
    DataRCV_HLW = 0;
    memset(buffer_HLW, 0, sizeof(buffer_HLW));
    memset(&calc, 0, sizeof(calc));
    memset(&merge, 0, sizeof(merge));
}

bool HLW_ReadData(void)
{
    uint8_t byte;
    uint32_t start_time = HAL_GetTick();

    indx_HLW = 0;
    memset(buffer_HLW, 0, sizeof(buffer_HLW));

    while ((HAL_GetTick() - start_time) < HLW_TIMEOUT_MS)
    {
        HAL_StatusTypeDef status = HAL_UART_Receive(&hlpuart1, &byte, 1, 10);
        if (status == HAL_OK)
        {
            if (byte == 0x55)
            {
                buffer_HLW[0] = byte;
                indx_HLW = 1;
            }
            else if (byte == 0x5A && indx_HLW == 1)
            {
                buffer_HLW[1] = byte;
                indx_HLW = 2;
            }
            else if (indx_HLW >= 2 && indx_HLW < 24)
            {
                buffer_HLW[indx_HLW] = byte;
                indx_HLW++;

                if (indx_HLW == 24)
                {
                    addr->VparamReg = ((uint32_t)buffer_HLW[2] << 16) |
                                      ((uint32_t)buffer_HLW[3] << 8) |
                                      buffer_HLW[4];

                    addr->VReg = ((uint32_t)buffer_HLW[5] << 16) |
                                 ((uint32_t)buffer_HLW[6] << 8) |
                                 buffer_HLW[7];

                    addr->IparamReg = ((uint32_t)buffer_HLW[8] << 16) |
                                      ((uint32_t)buffer_HLW[9] << 8) |
                                      buffer_HLW[10];

                    addr->IReg = ((uint32_t)buffer_HLW[11] << 16) |
                                 ((uint32_t)buffer_HLW[12] << 8) |
                                 buffer_HLW[13];

                    addr->PowparamReg = ((uint32_t)buffer_HLW[14] << 16) |
                                        ((uint32_t)buffer_HLW[15] << 8) |
                                        buffer_HLW[16];

                    addr->PowReg = ((uint32_t)buffer_HLW[17] << 16) |
                                   ((uint32_t)buffer_HLW[18] << 8) |
                                   buffer_HLW[19];

                    addr->PF = ((uint32_t)buffer_HLW[21] << 8) | buffer_HLW[22];

                    ptr->Voltage = (addr->VReg != 0) ?
                        ((float)addr->VparamReg * koefisienVoltage / (float)addr->VReg) : 0.0f;

                    ptr->Current = (addr->IReg != 0) ?
                        ((float)addr->IparamReg * koefisienCurrent / (float)addr->IReg) : 0.0f;

                    ptr->activePower = (addr->PowReg != 0) ?
                        ((float)addr->PowparamReg * koefisienPower / (float)addr->PowReg) : 0.0f;

                    ptr->apparentPower = ptr->Voltage * ptr->Current;

                    ptr->PF = (ptr->apparentPower != 0.0f) ?
                        (ptr->activePower / ptr->apparentPower) : 0.0f;

                    indx_HLW = 0;
                    return true;
                }
            }
            else
            {
                indx_HLW = 0;
            }
        }
    }

    indx_HLW = 0;
    return false;
}

bool HLW_IsDataValid(void)
{
    return (ptr->Voltage > 0.0f);
}

Param* HLW_GetData(void)
{
    return ptr;
}

// ============================================================================
//                        GPS FUNCTIONS - OPTIMIZED
// ============================================================================

/**
 * @brief Initialize GPS UART dengan prioritas rendah
 */
void UART_GPS_Init(void)
{
    gpsIndex = 0;
    gps_sentence_ready = false;
    gps_enabled = true;
    gps_fix_obtained = false;
    gps_fix_quality = 0;

    memset(gpsBuffer, 0, sizeof(gpsBuffer));
    memset(gga_string, 0, sizeof(gga_string));

    HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxData, 1);
}

/**
 * @brief Stop GPS reception (hemat CPU setelah dapat fix)
 */
void GPS_Stop(void)
{
    gps_enabled = false;
    HAL_UART_AbortReceive_IT(&huart2);  // Stop interrupt
}

/**
 * @brief Restart GPS reception jika diperlukan
 */
void GPS_Start(void)
{
    gps_enabled = true;
    gps_fix_obtained = false;
    gpsIndex = 0;
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxData, 1);
}

/**
 * @brief Check apakah GPS sudah mendapat fix valid
 */
bool GPS_HasFix(void)
{
    return gps_fix_obtained;
}

/**
 * @brief Get GPS fix quality (0=invalid, 1=GPS, 2=DGPS)
 */
uint8_t GPS_GetFixQuality(void)
{
    return gps_fix_quality;
}

/**
 * @brief Convert latitude dari NMEA format ke decimal degrees
 */
static void latitudeConv(void)
{
    if (gga_string[2][0] == '\0')
    {
        sendLat = 0.0f;
        return;
    }

    float temp = atof(gga_string[2]);
    int degree = (int)(temp / 100);
    float minutes = fmod(temp, 100.0f) / 60.0f;
    float latitude = (float)degree + minutes;

    if (gga_string[3][0] == 'S')
        latitude *= -1.0f;

    sendLat = latitude;
}

/**
 * @brief Convert longitude dari NMEA format ke decimal degrees
 */
static void longitudeConv(void)
{
    if (gga_string[4][0] == '\0')
    {
        sendLng = 0.0f;
        return;
    }

    float temp = atof(gga_string[4]);
    int degree = (int)(temp / 100);
    float minutes = fmod(temp, 100.0f) / 60.0f;
    float longitude = (float)degree + minutes;

    if (gga_string[5][0] == 'W')
        longitude *= -1.0f;

    sendLng = longitude;
}

/**
 * @brief Process GPS data - DIPANGGIL DARI MAIN LOOP, BUKAN ISR!
 *        Akan otomatis stop setelah mendapat fix valid
 *
 * @param auto_stop_on_fix  true = stop GPS setelah dapat fix
 */
void GPS_ProcessData(bool auto_stop_on_fix)
{
    // Skip jika tidak ada data baru atau GPS sudah disabled
    if (!gps_sentence_ready || !gps_enabled)
        return;

    // Reset flag
    gps_sentence_ready = false;

    // Hanya proses GPGGA sentence
    if (strncmp(gpsBuffer, "$GPGGA", 6) != 0)
        return;

    // Parse NMEA fields
    int varIdx = 0;
    int lenIdx = 0;
    int fieldIdx = 0;
    int bufLen = strlen(gpsBuffer);

    // Clear parsing buffer
    memset(new_string, 0, sizeof(new_string));

    // Parse comma-separated fields
    for (varIdx = 0; varIdx <= bufLen && fieldIdx < number_of_variable; varIdx++)
    {
        char c = gpsBuffer[varIdx];

        if (c == ',' || c == '\0' || c == '\r' || c == '\n' || c == '*')
        {
            new_string[fieldIdx][lenIdx] = '\0';
            fieldIdx++;
            lenIdx = 0;
        }
        else
        {
            if (lenIdx < string_length_size - 1)
                new_string[fieldIdx][lenIdx++] = c;
        }
    }

    // Verify it's GPGGA and copy to gga_string
    if (strcmp(new_string[0], "$GPGGA") == 0)
    {
        memcpy(gga_string, new_string, sizeof(gga_string));

        // Parse time (field 1): HHMMSS.sss
        if (gga_string[1][0] != '\0')
        {
            sscanf(gga_string[1], "%2d%2d%2d", &hh, &mm, &ss);
            hh = (hh + 7) % 24;  // UTC+7 untuk WIB
            snprintf(time_local, sizeof(time_local), "%02d:%02d:%02d", hh, mm, ss);
        }

        // Parse fix quality (field 6)
        // 0 = invalid, 1 = GPS fix, 2 = DGPS fix, etc.
        gps_fix_quality = 0;
        if (gga_string[6][0] != '\0')
        {
            gps_fix_quality = atoi(gga_string[6]);
        }

        // Convert coordinates jika ada fix
        if (gps_fix_quality >= 1)
        {
            latitudeConv();
            longitudeConv();

            // Validasi koordinat (Indonesia: Lat -11 to 6, Lng 95 to 141)
            if (sendLat != 0.0f && sendLng != 0.0f)
            {
                gps_fix_obtained = true;

                // Auto stop jika diminta dan sudah dapat fix
                if (auto_stop_on_fix)
                {
                    GPS_Stop();
                }
            }
        }
    }
}

/**
 * @brief Get current GPS coordinates
 */
void GPS_GetCoordinates(float *lat, float *lng)
{
    if (lat != NULL) *lat = sendLat;
    if (lng != NULL) *lng = sendLng;
}

/**
 * @brief Get current GPS time string
 */
const char* GPS_GetTimeString(void)
{
    return time_local;
}

// ============================================================================
//                        NVIC PRIORITY CONFIGURATION
// ============================================================================

/*** *
 * Panggil fungsi ini di main() sebelum UART_GPS_Init()
 */
void UART_ConfigurePriorities(void)
{
    // UART1 (STA Communication) - HIGH PRIORITY
    // PreemptPriority: 0-15, SubPriority: 0-15 (tergantung NVIC config)
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);  // Priority 1 (tinggi)

    // UART2 (GPS) - LOW PRIORITY
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);  // Priority 3 (lebih rendah)

    // Pastikan interrupt enabled
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

// ============================================================================
//                        UTILITY FUNCTIONS
// ============================================================================

void blinkLED(GPIO_TypeDef* port, uint16_t pin, uint8_t times, uint16_t delayMs)
{
    for (uint8_t i = 0; i < times; i++)
    {
        HAL_GPIO_TogglePin(port, pin);
        HAL_Delay(delayMs);
        HAL_GPIO_TogglePin(port, pin);
        HAL_Delay(delayMs);
    }
}

void Set_PWM1_Duty(uint8_t percent)
{
    if (percent > 100)
        percent = 100;

    pulse = (15999 * percent) / 100;
    dimming = 100 - percent;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
}
