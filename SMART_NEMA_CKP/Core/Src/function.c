/*
 * function.c
 *
 *  Created on: Dec 31, 2025
 *      Author: firza
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

uint32_t lastSensorTick = 0;
#define SENSOR_INTERVAL_MS  1000   // baca setiap 1 detik

// ==================== TEMPORARY BUFFER FOR JSON PARSING ====================
static char temp_value[32];

uint32_t delays = 5000;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;

//// ==================== VAC CONFIG ====================
uint32_t adc_value = 0;
float R1 = 10000.0f;        // R5 = 10k
float R2 = 1000.0f;         // R6 = 1k
float vref = 3.3f;
float vout = 0.0f;          // Tegangan di PA0
float vin_dc = 0.0f;        // Tegangan DC setelah rectifier
float vin_ac = 0.0f;        // Estimasi tegangan AC (Vrms)
uint32_t adc_vac_raw  = 0;
#define KALIBRASI_VAC  205.68f  // kalibrasi: 207.86 × (230.2V_asli / 235.4V_baca) = 203.27
float vac_fix = 0.0f;

int pulse = 15999;
int dimming = 0;
int dutytest = 100, relaytest = 0;

// ==================== SENSOR CT ARUS ====================
#define CURRENT_CAL     28.3f  // kalibrasi: 0.182A (40W/220V) / dbg_ct_vrms(0.00643V) = 28.3
#define CT_SAMPLES      200    // sampel per batch. 1 HAL_ADC_Start/Stop per 200 sample → ~0.8ms/call
#define OFFSET_SAMPLES  2000
#define CT_EMA_ALPHA    0.7f   // EMA smoothing: 0=beku, 1=tanpa filter. Time constant ~0.4 detik

uint32_t adc_ct_raw        = 0;
float ct_offsetVoltage     = 0.0f;
float ct_current_rms       = 0.0f;
float ct_fix               = 0.0f;

// ==================== DEBUG VARIABLES (pantau di Live Expressions) ====================
// Tambahkan semua variabel ini ke panel Live Expressions STM32CubeIDE
uint32_t dbg_ct_offset_counts  = 0;    // Seharusnya ~2040 (= 1.65V/3.3V * 4095). Jika <<2040 = offset salah
uint32_t dbg_ct_raw_min        = 4095; // Min ADC CT per Sensor_ReadAll. Jika ≈0 = clipping bawah
uint32_t dbg_ct_raw_max        = 0;    // Max ADC CT per Sensor_ReadAll. Jika ≈4095 = clipping atas
float    dbg_ct_peak_voltage   = 0.0f; // Amplitudo puncak sinyal AC CT (Volt). Misal 0.33V untuk 10A@33Ω
float    dbg_ct_vrms           = 0.0f; // Vrms sebelum dikali CURRENT_CAL. Bagi CURRENT_CAL = arus
uint16_t dbg_ct_clipped        = 0;    // Jumlah sampel yang clipping (0 atau 4095). Idealnya = 0
uint16_t dbg_sensor_calls_ps   = 0;    // Berapa kali Sensor_ReadAll dipanggil per detik (untuk cek loop rate)
uint32_t dbg_ct_total_samples  = 0;    // Total sampel yang masuk ke akumulator per detik (target: >50.000)
static uint16_t dbg_sensor_call_cnt = 0;
static uint32_t dbg_sensor_tick_ref = 0;

// ===== AKUMULATOR 1 DETIK — kunci stabilitas RMS =====
// Setiap Sensor_ReadAll menambahkan BATCH (200 sampel) ke sini.
// Sensor_Update setiap 1 detik menghitung RMS dari SEMUA sampel yang terkumpul.
static float    s_ct_sq_accum   = 0.0f; // Σ(ac²)  — untuk E[ac²]
static float    s_ct_sum_accum  = 0.0f; // Σ(ac)   — untuk E[ac], dipakai koreksi DC dinamis
static uint32_t s_ct_sq_count   = 0;    // jumlah sampel CT terakumulasi
static float    s_vac_accum     = 0.0f; // Σ(raw VAC) dari semua sampel
static uint32_t s_vac_count     = 0;    // jumlah sampel VAC terakumulasi

// ==================== GPS CONFIG ====================
#define number_of_variable  50
#define string_length_size  15

char time_local[10];
char new_string[number_of_variable][string_length_size];
char gga_string[number_of_variable][string_length_size];

volatile uint8_t rxData;
char gpsBuffer[256];
volatile uint16_t gpsIndex = 0;
int hh, mm, ss;

float sendLat = 0.0f;
float sendLng = 0.0f;

// ===== GPS STATE MANAGEMENT =====
volatile bool gps_sentence_ready = false;
volatile bool gps_enabled = true;
bool gps_fix_obtained = false;
uint8_t gps_fix_quality = 0;

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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint32_t current_tick = HAL_GetTick();

        if ((current_tick - last_rx_tick) > 100 && rx_index > 0)
        {
            rx_index = 0;
            memset(rxBuffer, 0, RX_BUFFER_STA);
        }
        last_rx_tick = current_tick;

        if (rx_index < RX_BUFFER_STA - 1)
        {
            rxBuffer[rx_index++] = rx_byte;
            rxBuffer[rx_index] = '\0';

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
        if (gps_enabled)
        {
            if (gpsIndex < sizeof(gpsBuffer) - 1)
            {
                gpsBuffer[gpsIndex++] = rxData;

                if (rxData == '\n')
                {
                    gpsBuffer[gpsIndex] = '\0';
                    gps_sentence_ready = true;  // Flag untuk main loop
                    gpsIndex = 0;
                }
            }
            else
            {
                gpsIndex = 0;
            }

            HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxData, 1);
        }
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
//    HLW_ReadData();
    SendDataResponse();
}

void Handle_G(int pwm_value)
{
    blinkLED(LED3_MAC_Port, LED3_MAC_Pin, 2, 30);
    dutytest = pwm_value;
    Set_PWM1_Duty((uint8_t)pwm_value);
//    HLW_ReadData();
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
//    HLW_ReadData();
    SendDataResponse();
}

// ============================================================================
//                        SEND RESPONSE
// ============================================================================

void SendDataResponse(void)
{
    char response[128];

    snprintf(response, sizeof(response),
//             "\"DATA\":{\"UID\":\"%s\",\"STA\":\"%s\",\"V\":\"%.1f\",\"I\":\"%.2f\",\"P\":\"%.1f\",\"PWM\":\"%d\"}",
            "\"DATA\":{\"UID\":\"%s\",\"STA\":\"%s\",\"V\":\"%.1f\",\"I\":\"%.2f\",\"P\":40,\"PWM\":\"%d\"}",
             mac_cco,
             mac_sta,
             vin_ac,
             ct_fix,
//             ptr->activePower,
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
//                        VAC _ CT FUNCTIONS
// ============================================================================

//void VAC_Read(void)
//{
//    extern ADC_HandleTypeDef hadc1;
//
//    uint32_t adc_sum = 0;
//
//    // Ambil 20 sample, rata-rata untuk stabilkan pembacaan
//    for (uint8_t i = 0; i < 40; i++)
//    {
//        HAL_ADC_Start(&hadc1);
//        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
//        {
//            adc_sum += HAL_ADC_GetValue(&hadc1);
//        }
//        HAL_Delay(2);
//    }
//
//    adc_value = adc_sum / 40;
//    vout   = ((float)adc_value / 4095.0f) * vref;
//    vin_ac = vout * KALIBRASI_VAC;
//
//
////    HAL_ADC_Start(&hadc1);
////    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
////    {
////        adc_value = HAL_ADC_GetValue(&hadc1);
////        vout = ((float)adc_value / 4095.0f) * vref;
////        vin_dc = vout * ((R1 + R2) / R2);
////        vin_ac = vout * 209.05f;
////    }
//}


// ============================================================================
//                     SENSOR VAC + CT ARUS (ADC 2 CHANNEL)
// ============================================================================

// Baca kedua channel ADC secara bergantian (Rank1 → Rank2)
static void ADC_ReadBoth(uint32_t *vac_raw, uint32_t *ct_raw)
{
    extern ADC_HandleTypeDef hadc1;

    // Rank 1 → Channel 1 (VAC)
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        *vac_raw = HAL_ADC_GetValue(&hadc1);

    // Rank 2 → Channel 11 (CT)
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        *ct_raw = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);
}

// Hitung offset CT — panggil SEKALI saat startup TANPA BEBAN
void CT_CalculateOffset(void)
{
    extern ADC_HandleTypeDef hadc1;
    uint32_t sum = 0;
    uint32_t dummy_vac = 0;
    uint32_t ct_sample = 0;

    for (int i = 0; i < OFFSET_SAMPLES; i++)
    {
        ADC_ReadBoth(&dummy_vac, &ct_sample);
        sum += ct_sample;
    }

    dbg_ct_offset_counts = sum / OFFSET_SAMPLES;  // DEBUG: pantau di Live Expressions
    ct_offsetVoltage = ((float)dbg_ct_offset_counts / 4095.0f) * 3.3f;
}

// Sensor_ReadAll — OPTIMIZED: hanya 1 HAL_ADC_Start/Stop per 200-sample batch.
// Trigger ulang ADC via register langsung (bypass overhead HAL per-sample).
// Estimasi waktu: ~0.8ms/call → ~700 call/detik → ~140.000 sampel/detik ≈ 140 siklus 50Hz/detik.
void Sensor_ReadAll(void)
{
    extern ADC_HandleTypeDef hadc1;
    uint32_t vac_raw_buf = 0;
    uint32_t ct_raw_buf  = 0;
    uint32_t poll_cnt;

    uint32_t loc_min  = 4095;
    uint32_t loc_max  = 0;
    uint16_t loc_clip = 0;
    float    loc_peak = 0.0f;

    // Bersihkan flag ADC yang mungkin tersisa dari konversi sebelumnya
    hadc1.Instance->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;

    // Satu kali HAL_ADC_Start untuk batch ini (urus state machine HAL)
    HAL_ADC_Start(&hadc1);

    for (int i = 0; i < CT_SAMPLES; i++)
    {
        // Tunggu Rank 1 (CH1 - VAC) selesai. Timeout 10000 iterasi (~60µs @ 168MHz)
        poll_cnt = 10000;
        while (!(hadc1.Instance->ISR & ADC_ISR_EOC) && --poll_cnt);
        vac_raw_buf = hadc1.Instance->DR;   // baca DR → otomatis clear EOC

        // Tunggu Rank 2 (CH11 - CT) selesai
        poll_cnt = 10000;
        while (!(hadc1.Instance->ISR & ADC_ISR_EOC) && --poll_cnt);
        ct_raw_buf = hadc1.Instance->DR;    // baca DR → otomatis clear EOC

        // Trigger sequence berikutnya via register langsung (lewati overhead HAL)
        // Lakukan untuk semua kecuali sampel terakhir
        if (i < CT_SAMPLES - 1)
        {
            hadc1.Instance->ISR = ADC_ISR_EOS | ADC_ISR_EOC; // clear EOS dan EOC
            hadc1.Instance->CR |= ADC_CR_ADSTART;             // trigger konversi berikutnya
        }

        // Masukkan ke akumulator 1-detik
        s_vac_accum += (float)vac_raw_buf;
        s_vac_count++;

        float ct_voltage = (float)ct_raw_buf * 3.3f / 4095.0f;
        float ac         = ct_voltage - ct_offsetVoltage;
        s_ct_sq_accum   += ac * ac;  // Σ(ac²) — untuk E[ac²]
        s_ct_sum_accum  += ac;       // Σ(ac)  — untuk koreksi DC via rumus variance
        s_ct_sq_count++;

        // Debug tracking per-batch
        if (ct_raw_buf < loc_min) loc_min = ct_raw_buf;
        if (ct_raw_buf > loc_max) loc_max = ct_raw_buf;
        if (ct_raw_buf == 0 || ct_raw_buf >= 4090) loc_clip++;
        float abs_ac = (ac < 0.0f) ? -ac : ac;
        if (abs_ac > loc_peak) loc_peak = abs_ac;
    }

    // Satu kali HAL_ADC_Stop untuk batch ini
    HAL_ADC_Stop(&hadc1);

    dbg_ct_raw_min      = loc_min;
    dbg_ct_raw_max      = loc_max;
    dbg_ct_clipped      = loc_clip;
    dbg_ct_peak_voltage = loc_peak;

    dbg_sensor_call_cnt++;
    uint32_t now_tick = HAL_GetTick();
    if ((now_tick - dbg_sensor_tick_ref) >= 1000)
    {
        dbg_sensor_calls_ps = dbg_sensor_call_cnt;
        dbg_sensor_call_cnt = 0;
        dbg_sensor_tick_ref = now_tick;
    }
}

// Sensor_Update — hitung RMS dari ~140.000 sampel (140 siklus 50Hz) tiap 1 detik.
// Tambahan EMA filter untuk haluskan variasi residual noise.
void Sensor_Update(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - lastSensorTick) >= SENSOR_INTERVAL_MS)
    {
        lastSensorTick = now;

        // --- VAC: rata-rata dari semua sampel 1 detik ---
        if (s_vac_count > 0)
        {
            adc_vac_raw = (uint32_t)(s_vac_accum / (float)s_vac_count);
            vout    = ((float)adc_vac_raw / 4095.0f) * 3.3f;
            vin_ac  = vout * KALIBRASI_VAC;
            vac_fix = vin_ac;
        }

        // --- CT: true AC RMS via rumus Variance, bebas dari DC bias ---
        // Vrms_AC = sqrt(E[ac²] - E[ac]²)
        // E[ac²] = s_ct_sq_accum / N → sudah ada
        // E[ac]  = s_ct_sum_accum / N → digunakan untuk koreksi DC dinamis
        // Ini TIDAK memerlukan ct_offsetVoltage yang akurat — DC apapun terkoreksi otomatis
        if (s_ct_sq_count > 0)
        {
            float N         = (float)s_ct_sq_count;
            float mean_ac   = s_ct_sum_accum / N;          // E[ac] — komponen DC residual
            float mean_sq   = s_ct_sq_accum  / N;          // E[ac²]
            float variance  = mean_sq - mean_ac * mean_ac; // Var = E[ac²] - E[ac]²
            if (variance < 0.0f) variance = 0.0f;          // proteksi float rounding

            dbg_ct_vrms = sqrtf(variance);  // True AC Vrms (DC sudah dikoreksi)

            // Noise floor: setelah koreksi DC, threshold lebih kecil dari sebelumnya
            // Atur CT_NOISE_FLOOR berdasarkan nilai dbg_ct_vrms saat lampu mati
            #define CT_NOISE_FLOOR  0.001f  // Volt. Nilai dbg_ct_vrms saat lampu benar-benar mati
            float vrms_clean = (dbg_ct_vrms > CT_NOISE_FLOOR) ? dbg_ct_vrms : 0.0f;

            // Koreksi duty-cycle: CT mengukur I_rms ∝ √duty, namun kita inginkan I ∝ duty (linier).
            // Dengan mengalikan lagi ×√(duty/max), hasilnya menjadi: I ∝ √duty × √duty = duty.
            // Efek: 10% input (physical 16.5%) → duty_factor=0.46 → I = 0.082A×0.46 ≈ 0.037A
            //        3% input (physical  8.2%) → lampu mati (di bawah ignition) → vrms_clean=0 → I=0
            //       100% input (physical 79%) → duty_factor=1.0 → I tidak berubah (recalibrate jika perlu)
            #define LAMP_MAX_D  0.79f
            float physical_duty = (float)pulse / 15999.0f;
            float duty_factor   = (physical_duty > 0.0f) ? sqrtf(physical_duty / LAMP_MAX_D) : 0.0f;
            if (duty_factor > 1.0f) duty_factor = 1.0f;

            ct_current_rms = vrms_clean * CURRENT_CAL * duty_factor;

            // EMA: α=0.7 → response ~2-3 detik setelah perubahan beban
            ct_fix = ct_fix * (1.0f - CT_EMA_ALPHA) + ct_current_rms * CT_EMA_ALPHA;

            dbg_ct_total_samples = s_ct_sq_count; // pantau: target >100.000
        }

        // Reset akumulator untuk window 1 detik berikutnya
        s_ct_sq_accum  = 0.0f;
        s_ct_sum_accum = 0.0f;
        s_ct_sq_count  = 0;
        s_vac_accum    = 0.0f;
        s_vac_count    = 0;
    }
}
// ============================================================================
//                        GPS FUNCTIONS - OPTIMIZED
// ============================================================================


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


void GPS_Stop(void)
{
    gps_enabled = false;
    HAL_UART_AbortReceive_IT(&huart2);  // Stop interrupt
}


void GPS_Start(void)
{
    gps_enabled = true;
    gps_fix_obtained = false;
    gpsIndex = 0;
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxData, 1);
}


bool GPS_HasFix(void)
{
    return gps_fix_obtained;
}


uint8_t GPS_GetFixQuality(void)
{
    return gps_fix_quality;
}


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

void GPS_ProcessData(bool auto_stop_on_fix)
{

    if (!gps_sentence_ready || !gps_enabled)
        return;

    // Reset flag
    gps_sentence_ready = false;


    if (strncmp(gpsBuffer, "$GPGGA", 6) != 0)
        return;


    int varIdx = 0;
    int lenIdx = 0;
    int fieldIdx = 0;
    int bufLen = strlen(gpsBuffer);

    memset(new_string, 0, sizeof(new_string));

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


    if (strcmp(new_string[0], "$GPGGA") == 0)
    {
        memcpy(gga_string, new_string, sizeof(gga_string));

        if (gga_string[1][0] != '\0')
        {
            sscanf(gga_string[1], "%2d%2d%2d", &hh, &mm, &ss);
            hh = (hh + 7) % 24;  // UTC+7 untuk WIB
            snprintf(time_local, sizeof(time_local), "%02d:%02d:%02d", hh, mm, ss);
        }

        gps_fix_quality = 0;
        if (gga_string[6][0] != '\0')
        {
            gps_fix_quality = atoi(gga_string[6]);
        }

        if (gps_fix_quality >= 1)
        {
            latitudeConv();
            longitudeConv();

            if (sendLat != 0.0f && sendLng != 0.0f)
            {
                gps_fix_obtained = true;

                if (auto_stop_on_fix)
                {
                    GPS_Stop();
                }
            }
        }
    }
}

void GPS_GetCoordinates(float *lat, float *lng)
{
    if (lat != NULL) *lat = sendLat;
    if (lng != NULL) *lng = sendLng;
}

const char* GPS_GetTimeString(void)
{
    return time_local;
}

// ============================================================================
//                        NVIC PRIORITY CONFIGURATION
// ============================================================================

void UART_ConfigurePriorities(void)
{

    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);

    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);

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
    if (percent > 100) percent = 100;

    int corrected_pulse;
    if (percent == 0)
    {
        corrected_pulse = 0;  // Mati total
    }
    else
    {
        /*
         * Gamma correction + pembatasan range efektif lampu:
         *
         * Mengapa dibatasi LAMP_MAX_DUTY = 79%?
         *   C3 = 10µF pada rangkaian CT memiliki τ = ~330µs. Pada duty fisik
         *   > 79%, OFF-time per siklus PWM (95µs × 21% = 20µs) << τ, sehingga
         *   C3 tidak sempat discharge → lampu dan sensor tidak bisa membedakan
         *   duty 79% vs 100%. Dengan membatasi maksimum ke 79%, input 100%
         *   tetap menghasilkan kecerahan maksimum tanpa masuk zona saturasi.
         *
         * Tabel referensi setelah perubahan (input % → duty fisik):
         *    1%  →   9%   (mendekati minimum lampu ~13%)
         *    2%  →  13%   (batas bawah lampu menyala)
         *   10%  →  25%
         *   30%  →  44%
         *   50%  →  56%
         *   70%  →  68%
         *   80%  →  72%
         *  100%  →  79%   (maksimum efektif)
         */
        const float LAMP_MAX_DUTY = 0.79f;   // duty fisik maksimum yang masih responsif

        float norm      = (float)percent / 100.0f;
        float gamma_out = powf(norm, 0.68f) * LAMP_MAX_DUTY;  // gamma 0.68: 3%→8%, 10%→16%, 100%→79%
        corrected_pulse = (int)(gamma_out * 15999.0f + 0.5f);
        if (corrected_pulse < 1)     corrected_pulse = 1;
        if (corrected_pulse > 15999) corrected_pulse = 15999;
    }

    pulse   = corrected_pulse;
    dimming = 100 - percent;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)corrected_pulse);
}
