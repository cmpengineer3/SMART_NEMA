#include "uart_cco.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

// ============================================================================
//                         BUFFER VARIABLES
// ============================================================================

char rxBuffer[RX_BUFFER_CCO];           // Buffer untuk receive (monitoring)
char recvBuffer[RX_BUFFER_CCO];         // Buffer untuk proses (parsing)
char txBuffer[200];
char mac_cco[13];

char uart_tx_last[128] = {0};
uint16_t uart_tx_length = 0;

static uint8_t rx_byte;
static volatile uint16_t rx_index = 0;
static volatile uint32_t last_rx_tick_cco = 0;

// Flag untuk menandakan ada +RECV siap diproses
volatile bool recv_data_ready = false;

const int max_retry = 3;
int attempt_debug = 0;

uint32_t delays = 5000;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

// ============================================================================
//                         NODE VARIABLES
// ============================================================================

STA_Node mac_sta_list[250];
uint16_t mac_sta_count = 0;

char mac_sta_new_list[250][13];
uint16_t mac_sta_new_count = 0;

char mac_wh_list[250][13];
uint16_t mac_wh_count = 0;

char mac_wh_new_list[250][13];
uint16_t mac_wh_new_count = 0;
//===========================================PENGIRIMAN DAN PENERIMAAN DATA===========================
void UART_CCO_ClearBuffer(void)
{
    memset(rxBuffer, 0, RX_BUFFER_CCO);
    rx_index = 0;
}

void UART_CCO_ClearRecvBuffer(void)
{
    memset(recvBuffer, 0, RX_BUFFER_CCO);
    recv_data_ready = false;
}

void UART_CCO_ClearNewTopoList(void)
{
    mac_sta_new_count = 0;
    for (int i = 0; i < 250; i++)
    	mac_sta_new_list[i][0] = '\0';
}

void UART_CCO_Init(void)
{
    UART_CCO_ClearBuffer();
    UART_CCO_ClearRecvBuffer();
    memset(mac_cco, 0, sizeof(mac_cco));
    last_rx_tick_cco = 0;

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/* callback interrupt */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint32_t current_tick = HAL_GetTick();

        // Auto clear jika jeda > 100ms (mencegah data lama menumpuk)
        if ((current_tick - last_rx_tick_cco) > 100 && rx_index > 0)
        {
            rx_index = 0;
            memset(rxBuffer, 0, RX_BUFFER_CCO);
        }
        last_rx_tick_cco = current_tick;

        // Simpan byte ke rxBuffer
        if (rx_index < RX_BUFFER_CCO - 1)
        {
            rxBuffer[rx_index++] = rx_byte;
            rxBuffer[rx_index] = '\0';

            // ══════════════════════════════════════════════════════════════
            // DETECT +RECV: dan COPY ke recvBuffer (SAMA SEPERTI STA)
            // ══════════════════════════════════════════════════════════════
            if (rx_byte == '\n' && strstr(rxBuffer, "+RECV:") != NULL)
            {
                // Copy ke recvBuffer jika belum ada data pending
                if (!recv_data_ready)
                {
                    strcpy(recvBuffer, rxBuffer);
                    recv_data_ready = true;
                }

                // Clear rxBuffer setelah copy
                rx_index = 0;
                memset(rxBuffer, 0, RX_BUFFER_CCO);
            }
        }
        else
        {
            // Buffer full, clear
            rx_index = 0;
            memset(rxBuffer, 0, RX_BUFFER_CCO);
        }

        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
    else if (huart->Instance == USART2)
    {
        MCU2_UART_RxCallback();
    }
}


void UART_CCO_SendRaw(const char *s)
{
    // Simpan data TX untuk Live Expression
    strncpy(uart_tx_last, s, sizeof(uart_tx_last) - 1);
    uart_tx_last[sizeof(uart_tx_last) - 1] = '\0';
    uart_tx_length = strlen(s);

    // Kirim data ke PLC
    HAL_UART_Transmit(&huart1, (uint8_t*)s, uart_tx_length, 1000);
}

void UART_CCO_SendAT(const char *cmd)
{
    char t[128];
    snprintf(t, sizeof(t), "%s\r\n", cmd);
    UART_CCO_SendRaw(t);
}

bool UART_CCO_WaitFor(const char *keyword, uint32_t timeout)
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

bool UART_CCO_SendCmdWait(const char *cmd, const char *urc, uint32_t t_ok, uint32_t t_urc)
{
    UART_CCO_ClearBuffer();
    UART_CCO_SendAT(cmd);

    if (!UART_CCO_WaitFor("OK", t_ok))
        return false;

    if (urc == NULL) return true;

    if (!UART_CCO_WaitFor(urc, t_urc))
        return false;

    /* khusus urc MAC */
    if (strstr(urc, "+MAC") != NULL)
    {
        char *p = strstr(rxBuffer, "+MAC:");
        if (p)
            strncpy(mac_cco, p + 5, sizeof(mac_cco) - 1);
    }

    //khusus untuk TOPOINFO
    if (strstr(urc, "+TOPOINFO") != NULL)
    {
        char buffer_copy[RX_BUFFER_CCO];
        strcpy(buffer_copy, rxBuffer);

        char *line = strtok(buffer_copy, "\r\n");

        while (line)
        {
            if (strstr(line, "+TOPOINFO:"))
                UART_CCO_ParseTopoInfo(line);

            line = strtok(NULL, "\r\n");
        }
    }
    //khusus untuk WHINFO
    if (strstr(urc, "+WHINFO") != NULL)
    {
        char buffer_copy[RX_BUFFER_CCO];
        strcpy(buffer_copy, rxBuffer);

        char *line = strtok(buffer_copy, "\r\n");

        while (line)
        {
            if (strstr(line, "+WHINFO:"))
            	UART_CCO_ParseWHInfo(line);

            line = strtok(NULL, "\r\n");
        }
    }

    return true;
}

//============================================= TOPOINFO : MAC STA AKTIF===============================

void UART_CCO_ParseTopoInfo(char *line)
{
    char mac[13] = {0};

    // format: +TOPOINFO:XXXXXXXXXXXX,...
    if (sscanf(line, "+TOPOINFO:%12[^,]", mac) == 1)
    {
    	UART_CCO_AddNewTopoMAC(mac); // menulis ke list baru
;
    }
}


void TOPOINFO(void)
{
	int attempt = 0;               // dipakai untuk retry logic
    bool ok = false;

    while (attempt < max_retry && !ok)
    {
        UART_CCO_ClearNewTopoList();

        ok = UART_CCO_SendCmdWait("AT+TOPOINFO=1,250", "+TOPOINFO:", 2000, 3000);

        if (!ok)
        {
            attempt++;
            attempt_debug++;
            HAL_Delay(300);   // beri waktu PLC bernapas
        }
    }

    if (ok)
    {
    	attempt=0;
    	attempt_debug=0;
        UART_CCO_SyncTopoList();
    }
}


void UART_CCO_AddNewTopoMAC(const char *mac)
{
    // FILTER prefix
    if (!(mac[0] == 'F' && mac[1] == 'F' && mac[2] == 'F'&& mac[3] == 'F'))
        return;

    // CEK DUPLIKAT di NEW LIST
    for (int i = 0; i < mac_sta_new_count; i++)
    {
        if (strncmp(mac_sta_new_list[i], mac, 12) == 0)
            return;
    }

    if (mac_sta_new_count >= 250)
        return;

    strncpy(mac_sta_new_list[mac_sta_new_count], mac, 12);
    mac_sta_new_list[mac_sta_new_count][12] = '\0';

//    snprintf(mac_sta_new_list[mac_sta_new_count].mac_sta,
//             sizeof(mac_sta_new_list[mac_sta_new_count].mac_sta),
//             "%s", mac);

//    // INIT VALUE
//    mac_sta_new_list[mac_sta_new_count].v = 0;
//    mac_sta_new_list[mac_sta_new_count].i = 0;
//    mac_sta_new_list[mac_sta_new_count].p = 0;
//    mac_sta_new_list[mac_sta_new_count].relay = 0;
//    mac_sta_new_list[mac_sta_new_count].pwm = 0;
//    mac_sta_new_list[mac_sta_new_count].cond = false; // default

    mac_sta_new_count++;
}


void UART_CCO_SyncTopoList(void)
{
    // Hapus isi list lama
    mac_sta_count = 0;

    // Copy semua dari new_list ke main_list
    for (int i = 0; i < mac_sta_new_count; i++)
    {
    	// Gunakan strncpy untuk keamanan buffer
    	strncpy(mac_sta_list[i].mac_sta, mac_sta_new_list[i], 12);

    	// **PASTIKAN NULL TERMINATOR ADA**
    	// mac_sta[12] adalah posisi ke-13, yang merupakan null terminator
    	mac_sta_list[i].mac_sta[12] = '\0';
    }

    mac_sta_count = mac_sta_new_count;
}

//=========================== WH INFO + WH ADD=================================
//void UART_CCO_SendWHADD_FromList(void)
//{
//    if (mac_sta_count == 0)
//        return;
//
//    char mac_buffer[4000] = {0};   // buffer besar untuk semua MAC
//    char cmd[4200] = {0};
//
//    // Bangun string "MAC1;MAC2;MAC3;..."
//    for (int i = 0; i < mac_sta_count; i++)
//    {
//        strcat(mac_buffer, mac_sta_list[i]);
//        if (i < mac_sta_count - 1)
//            strcat(mac_buffer, ";");
//    }
//
//    // Bangun perintah AT lengkap
//    snprintf(cmd, sizeof(cmd),
//             "AT+WHADD=%d,\"%s\"",
//             mac_sta_count, mac_buffer);
//
//    // Kirim perintah AT
//    UART_CCO_SendCmdWait(cmd, "OK", 2000, 2000);
//}


void UART_CCO_ClearNewWHList(void)
{
    mac_wh_new_count = 0;
    for (int i = 0; i < 250; i++)
        mac_wh_new_list[i][0] = '\0';
}

void UART_CCO_ParseWHInfo(char *line)
{
    char mac[13] = {0};

    if (sscanf(line, "+WHINFO:%12[^,]", mac) == 1)
    {
        UART_CCO_AddNewWHMAC(mac);
    }
}

void UART_CCO_AddNewWHMAC(const char *mac)
{
    // cek duplikat
    for (int i = 0; i < mac_wh_new_count; i++)
    {
        if (strncmp(mac_wh_new_list[i], mac, 12) == 0)
            return;
    }

    if (mac_wh_new_count >= 250)
        return;

    strncpy(mac_wh_new_list[mac_wh_new_count], mac, 12);
    mac_wh_new_list[mac_wh_new_count][12] = '\0';
    mac_wh_new_count++;
}

void UART_CCO_SyncWHList(void)
{
    mac_wh_count = 0;

    for (int i = 0; i < mac_wh_new_count; i++)
        strncpy(mac_wh_list[i], mac_wh_new_list[i], 13);

    mac_wh_count = mac_wh_new_count;
}

void WHINFO(void)
{
	int attempt = 0;               // dipakai untuk retry logic
    bool ok = false;

    while (attempt < max_retry && !ok)
    {
    	UART_CCO_ClearNewWHList();

        ok = UART_CCO_SendCmdWait("AT+WHINFO=1,250", "+WHINFO:", 2000, 3000);

        if (!ok)
        {
            attempt++;
            attempt_debug++;
            HAL_Delay(300);   // beri waktu PLC bernapas
        }
    }

    if (ok)
    {
    	attempt=0;
    	attempt_debug=0;
    	UART_CCO_SyncWHList();
    }
}

// ============================================================================
//                    AT+SENDEX FUNCTION
// ============================================================================

bool CCO_SendData_SENDEX(const char *mac_dest, const char *payload)
{
    char cmd[64];
    uint16_t payload_len = strlen(payload);

    // 1. Clear buffer
    UART_CCO_ClearBuffer();

    // 2. Kirim AT+SENDEX command
    snprintf(cmd, sizeof(cmd), "AT+SENDEX=%s,%u", mac_dest, payload_len);
    UART_CCO_SendAT(cmd);

    // 3. Tunggu prompt ">"
    if (!UART_CCO_WaitFor(">", 2000))
    {
//        char debug[128];
//        snprintf(debug, sizeof(debug), "[CCO_ERR] No '>' prompt for %s\r\n", mac_dest);
//        HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 1000);
        return false;
    }

    // 4. Kirim payload
    UART_CCO_SendRaw(payload);
    HAL_Delay(200);

    // 5. Tunggu "OK"
    if (!UART_CCO_WaitFor("OK", 2000))
    {
//        char debug[128];
//        snprintf(debug, sizeof(debug), "[CCO_ERR] No OK for %s\r\n", mac_dest);
//        HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 1000);
        return false;
    }

    // 6. Clear rxBuffer, siap terima +RECV
    //    Response +RECV akan di-copy ke recvBuffer oleh callback
    UART_CCO_ClearBuffer();

    return true;
}


// ============================================================================
//                    JSON HELPER FUNCTION
// ============================================================================

/**
 * @brief Extract value dari JSON berdasarkan key
 *
 * Contoh: ExtractJsonValue(json, "V", value, 16) → value = "220.5"
 */
bool ExtractJsonValue(const char *json, const char *key, char *value, size_t max_len)
{
    if (json == NULL || key == NULL || value == NULL)
        return false;

    value[0] = '\0';

    // Cari "KEY":
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":", key);

    char *pos = strstr(json, search);
    if (pos == NULL)
        return false;

    // Skip ke setelah ":"
    pos += strlen(search);

    // Skip whitespace
    while (*pos == ' ' || *pos == '\t') pos++;

    // Cek apakah value dalam quotes
    if (*pos == '"')
    {
        pos++;  // Skip opening quote

        size_t i = 0;
        while (*pos != '"' && *pos != '\0' && i < max_len - 1)
        {
            value[i++] = *pos++;
        }
        value[i] = '\0';
    }
    else
    {
        // Value tanpa quotes (number)
        size_t i = 0;
        while (*pos != ',' && *pos != '}' && *pos != '\0' && i < max_len - 1)
        {
            value[i++] = *pos++;
        }
        value[i] = '\0';
    }

    return (strlen(value) > 0);
}


// ============================================================================
//                    PARSE +RECV LINE
// ============================================================================

/**
 * @brief Parse +RECV line dari recvBuffer
 *
 * Format: +RECV:FFFFFFFFFFFF,85,"DATA":{"UID":"xxx","STA":"yyy","V":"220.5",...},1
 */
void CCO_ParseRECV_Line(const char *line, char *mac_out, char *payload_out, size_t payload_max)
{
    mac_out[0] = '\0';
    payload_out[0] = '\0';

    // Cari +RECV:
    char *p = strstr(line, "+RECV:");
    if (p == NULL)
        return;

    p += 6;  // Skip "+RECV:"

    // Extract MAC (12 karakter pertama)
    strncpy(mac_out, p, 12);
    mac_out[12] = '\0';

    // Cari awal payload: "DATA":
    char *payload_start = strstr(p, "\"DATA\":");
    if (payload_start == NULL)
        return;

    // Cari akhir payload: }, sebelum flag
    char *payload_end = strstr(payload_start, "},");
    if (payload_end != NULL)
    {
        payload_end++;  // Include '}'
    }
    else
    {
        // Cari } terakhir
        payload_end = strrchr(payload_start, '}');
        if (payload_end != NULL)
            payload_end++;
    }

    if (payload_end == NULL)
        return;

    // Copy payload
    size_t payload_len = payload_end - payload_start;
    if (payload_len > 0 && payload_len < payload_max)
    {
        strncpy(payload_out, payload_start, payload_len);
        payload_out[payload_len] = '\0';
    }
}


// ============================================================================
//                    PARSE STA RESPONSE
// ============================================================================

/**
 * @brief Parse response JSON dari STA dan update node
 *
 * Format: "DATA":{"UID":"MAC_CCO","STA":"MAC_STA","V":"220.5","I":"1.25","P":"275.6","D":"50"}
 */
void CCO_ParseSTAResponse(const char *json)
{
    if (json == NULL || strlen(json) < 10)
        return;

    char sta_mac[16] = {0};
    char v_str[16] = {0};
    char i_str[16] = {0};
    char p_str[16] = {0};
    char d_str[16] = {0};

    // Extract STA (MAC pengirim)
    if (!ExtractJsonValue(json, "STA", sta_mac, sizeof(sta_mac)))
    {
//        char debug[128];
//        snprintf(debug, sizeof(debug), "[CCO_ERR] No STA in response\r\n");
//        HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 1000);
        return;
    }

    // Extract values
    ExtractJsonValue(json, "V", v_str, sizeof(v_str));
    ExtractJsonValue(json, "I", i_str, sizeof(i_str));
    ExtractJsonValue(json, "P", p_str, sizeof(p_str));
    ExtractJsonValue(json, "D", d_str, sizeof(d_str));

    // Convert
    float v_val = (v_str[0] != '\0') ? atof(v_str) : 0.0f;
    float i_val = (i_str[0] != '\0') ? atof(i_str) : 0.0f;
    float p_val = (p_str[0] != '\0') ? atof(p_str) : 0.0f;
    uint8_t d_val = (d_str[0] != '\0') ? (uint8_t)atoi(d_str) : 0;

    // Cari node berdasarkan STA MAC
    uint16_t node_index;
    if (CCO_FindNodeByMAC(sta_mac, &node_index))
    {
        // Update data node
        mac_sta_list[node_index].v = (uint16_t)(v_val * 10);
        mac_sta_list[node_index].i = (uint16_t)(i_val * 100);
        mac_sta_list[node_index].p = (uint16_t)p_val;
        mac_sta_list[node_index].dim = d_val;
        mac_sta_list[node_index].cond = true;
        mac_sta_list[node_index].retry_count = 0;

//        char debug[128];
//        snprintf(debug, sizeof(debug),
//                 "[CCO_OK] STA %s: V=%.1f I=%.2f P=%.1f D=%u\r\n",
//                 sta_mac, v_val, i_val, p_val, d_val);
//        HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 1000);
    }
    else
    {
//        char debug[128];
//        snprintf(debug, sizeof(debug), "[CCO_WARN] STA %s not in list\r\n", sta_mac);
//        HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 1000);
    }
}


// ============================================================================
//                    WAIT AND PARSE +RECV (DARI recvBuffer)
// ============================================================================

/**
 * @brief Tunggu +RECV dan parse dari recvBuffer
 *
 * Callback akan copy +RECV ke recvBuffer dan set recv_data_ready = true
 * Fungsi ini hanya perlu cek flag dan parse dari recvBuffer
 */
bool CCO_WaitAndParse_RECV(const char *mac_expected, char *payload_out, size_t payload_size, uint32_t timeout)
{
    uint32_t start_time = HAL_GetTick();
    payload_out[0] = '\0';

    while ((HAL_GetTick() - start_time) < timeout)
    {
        // Cek apakah ada data +RECV di recvBuffer
        if (recv_data_ready)
        {
            // Parse dari recvBuffer (BUKAN rxBuffer!)
            char mac_from[13] = {0};
            char payload_temp[256] = {0};

            CCO_ParseRECV_Line(recvBuffer, mac_from, payload_temp, sizeof(payload_temp));

            // Debug: tampilkan data yang diterima
//            char debug[300];
//            snprintf(debug, sizeof(debug),
//                     "[CCO_RX] From %s, payload: %.100s\r\n",
//                     mac_from, payload_temp);
//            HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 1000);

            // Clear recvBuffer setelah diproses
            UART_CCO_ClearRecvBuffer();

            // Validasi MAC (jika diperlukan)
            if (mac_expected == NULL || strncmp(mac_from, mac_expected, 12) == 0)
            {
                strncpy(payload_out, payload_temp, payload_size - 1);
                payload_out[payload_size - 1] = '\0';
                return true;
            }
            else
            {
                // MAC tidak sesuai, tapi tetap proses
//                snprintf(debug, sizeof(debug),
//                         "[CCO_INFO] Response from %s (expected %s)\r\n",
//                         mac_from, mac_expected);
//                HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 1000);

                strncpy(payload_out, payload_temp, payload_size - 1);
                payload_out[payload_size - 1] = '\0';
                return true;
            }
        }

        HAL_Delay(10);
    }

    return false;  // Timeout
}


// ============================================================================
//                    NODE FUNCTIONS
// ============================================================================

bool CCO_FindNodeByMAC(const char *mac, uint16_t *index)
{
    for (uint16_t i = 0; i < mac_sta_count; i++)
    {
        if (strncmp(mac_sta_list[i].mac_sta, mac, 12) == 0)
        {
            *index = i;
            return true;
        }
    }
    return false;
}

void CCO_UpdateNodeData(uint16_t index, uint16_t v, uint16_t i, uint16_t dim,
                        const char *lat, const char *lng)
{
    if (index >= mac_sta_count)
        return;

    mac_sta_list[index].v = v;
    mac_sta_list[index].i = i;
    mac_sta_list[index].dim = dim;
    mac_sta_list[index].p = (uint16_t)((uint32_t)v * i / 10000);

    if (lat != NULL && strlen(lat) > 0)
        mac_sta_list[index].La = atof(lat);

    if (lng != NULL && strlen(lng) > 0)
        mac_sta_list[index].Lo = atof(lng);

    mac_sta_list[index].cond = true;
}


// ============================================================================
//                    POLLING FUNCTIONS
// ============================================================================

void CCO_PollAllSTA_SENDEX(void)
{
//    char debug[128];
//
//    snprintf(debug, sizeof(debug),
//             "[CCO_POLL] Start polling %u STAs\r\n", mac_sta_count);
//    HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 1000);

    for (uint16_t i = 0; i < mac_sta_count; i++)
    {
        bool success = false;
        uint8_t retry = 0;

        // Clear recvBuffer sebelum polling STA ini
        UART_CCO_ClearRecvBuffer();

        while (retry < POLLING_MAX_RETRY && !success)
        {
//            snprintf(debug, sizeof(debug),
//                     "[CCO_REQ] Poll STA[%u] %s (try %u)\r\n",
//                     i, mac_sta_list[i].mac_sta, retry + 1);
//            HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 1000);

            // Format payload RQ
            char payload[128];
            snprintf(payload, sizeof(payload),
                     "\"DATA\":{\"UID\":\"%s\",\"HEADER\":\"RQ\"}",
                     mac_cco);
            strncpy(txBuffer, payload, sizeof(txBuffer) - 1);
            txBuffer[sizeof(txBuffer) - 1] = '\0';

            // Kirim request
            if (CCO_SendData_SENDEX(mac_sta_list[i].mac_sta, payload))
            {
                // Tunggu response dari recvBuffer
                char response[256];
                if (CCO_WaitAndParse_RECV(mac_sta_list[i].mac_sta, response, sizeof(response), POLLING_TIMEOUT_MS))
                {
                    // Parse response
                    CCO_ParseSTAResponse(response);
                    success = true;
                }
                else
                {
                    retry++;
                    mac_sta_list[i].retry_count++;

//                    snprintf(debug, sizeof(debug),
//                             "[CCO_TIMEOUT] STA %s no response\r\n",
//                             mac_sta_list[i].mac_sta);
//                    HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 1000);
                }
            }
            else
            {
                retry++;
                HAL_Delay(200);
            }
        }

        if (!success)
        {
            mac_sta_list[i].cond = false;

//            snprintf(debug, sizeof(debug),
//                     "[CCO_FAIL] STA %s marked inactive\r\n",
//                     mac_sta_list[i].mac_sta);
//            HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 1000);
        }

        HAL_Delay(POLLING_DELAY_BETWEEN);
    }

//    snprintf(debug, sizeof(debug), "[CCO_POLL] Completed\r\n");
//    HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 1000);
}


void CCO_PollAllSTA_PWM(void)
{
//    char debug[128];
//
//    snprintf(debug, sizeof(debug),
//             "[CCO_PWM] Start PWM poll %u STAs\r\n", mac_sta_count);
//    HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 1000);

    for (uint16_t i = 0; i < mac_sta_count; i++)
    {
        bool success = false;
        uint8_t retry = 0;

        UART_CCO_ClearRecvBuffer();

        while (retry < POLLING_MAX_RETRY && !success)
        {
            int pwm_val = rand() % 101;

            char payload[128];
            snprintf(payload, sizeof(payload),
                     "\"DATA\":{\"UID\":\"%s\",\"HEADER\":\"G\",\"PWM\":\"%d\"}",
                     mac_cco, pwm_val);
            strncpy(txBuffer, payload, sizeof(txBuffer) - 1);
            txBuffer[sizeof(txBuffer) - 1] = '\0';

            if (CCO_SendData_SENDEX(mac_sta_list[i].mac_sta, payload))
            {
                char response[256];
                if (CCO_WaitAndParse_RECV(mac_sta_list[i].mac_sta, response, sizeof(response), POLLING_TIMEOUT_MS))
                {
                    CCO_ParseSTAResponse(response);
                    success = true;
                }
                else
                {
                    retry++;
                    mac_sta_list[i].retry_count++;
                }
            }
            else
            {
                retry++;
                HAL_Delay(200);
            }
        }

        if (!success)
        {
            mac_sta_list[i].cond = false;
        }

        HAL_Delay(POLLING_DELAY_BETWEEN);
    }

//    snprintf(debug, sizeof(debug), "[CCO_PWM] Completed\r\n");
//    HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 1000);
}


// ============================================================================
//                    USART2 OUTPUT FUNCTIONS
// ============================================================================

void USART2_SendNodeAsJSON(STA_Node* node)
{
    char buffer[200];

    float voltage = (float)node->v / 10.0f;
    float current = (float)node->i / 100.0f;

    int len = snprintf(buffer, sizeof(buffer),
             "{\"mac\":\"%s\",\"la\":%.6f,\"lo\":%.6f,"
             "\"v\":%.1f,\"i\":%.2f,\"p\":%u,"
             "\"relay\":%u,\"dim\":%u,\"cond\":%d}\r\n",
             node->mac_sta,
             node->La,
             node->Lo,
             voltage,
             current,
             node->p,
             node->relay,
             node->dim,
             node->cond ? 1 : 0);

    if (len >= sizeof(buffer))
        return;

    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000);
    while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);
    HAL_Delay(15);
}

void USART2_SendAllNodes_JSON(void)
{
    char start[] = "{\"nodes\":[\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)start, strlen(start), 1000);
    while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);
    HAL_Delay(10);

    for(uint16_t i = 0; i < mac_sta_count; i++)
    {
        USART2_SendNodeAsJSON(&mac_sta_list[i]);

        if(i < mac_sta_count - 1)
        {
            char comma[] = ",";
            HAL_UART_Transmit(&huart2, (uint8_t*)comma, 1, 100);
            while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);
        }
    }

    HAL_Delay(10);

    char end[] = "]}\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)end, strlen(end), 1000);
    while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);
}

//===================================================================================
void config (void)
{
	UART_CCO_SendCmdWait("+++", NULL, 1000, 0);
	HAL_Delay(delays);
	UART_CCO_SendCmdWait("AT+MAC?", "+MAC:", 1000, 2000);
	HAL_Delay(delays);
	UART_CCO_SendCmdWait("AT+MODE=2", "OK", 1000, 1000);
	HAL_Delay(delays);
	UART_CCO_SendCmdWait("AT+BLINDNV=5", "OK", 1000, 1000);
	HAL_Delay(delays);
	UART_CCO_SendCmdWait("AT+FREQ=5,10", "OK", 1000, 1000);
	HAL_Delay(delays);
	UART_CCO_SendCmdWait("AT+RST", "OK", 1000, 2000);
	HAL_Delay(delays);
}


void searching (void)
{
	UART_CCO_SendCmdWait("+++", NULL, 1000, 0);
	HAL_Delay(delays);
	int attempt = 0;
	bool ok = false;

	    while (attempt < max_retry && !ok)
	    {
	        // Perintah AT+WHSTATUS=0
	        ok = UART_CCO_SendCmdWait("AT+WHSTATUS=0", "OK", 1000, 1000);

	        if (!ok)
	        {
	            attempt++;
	            attempt_debug++;
	            HAL_Delay(300); // Beri waktu istirahat
	        }
	    }

	    if (ok)
	    {
	    	attempt=0;
	    	attempt_debug=0;
	    }
	HAL_Delay(delays);
	UART_CCO_ClearBuffer();
	TOPOINFO();

}

void Save_Node (void)
{
//	UART_CCO_SendCmdWait("+++", NULL, 1000, 0);
//	HAL_Delay(delays);
	UART_CCO_SendCmdWait("AT+MAC?", "+MAC:", 1000, 2000);
	HAL_Delay(delays);
	TOPOINFO();
	HAL_Delay(delays);
//	UART_CCO_SendCmdWait("AT+EXIT", "OK", 1000, 1000);
//	HAL_Delay(delays);
}

//void Send_GPIO_Toggle(void)
//{
//    static uint8_t gpio_state = 0;
//    char cmd[64];
//
//    snprintf(txBuffer, sizeof(txBuffer),
//             "AT+IOCTRL=FFFF00000001,0,%u",
//             gpio_state);
//
//    UART_CCO_SendAT(cmd);
//
//    gpio_state ^= 1;   // toggle 0 ↔ 1
//}
