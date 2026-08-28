#include "uart.h"
#include "main.h"
/* ── Internal state ───────────────────────────────────────────────────────── */
static UART_HandleTypeDef *s_huart  = NULL;
static uint8_t             rx_byte  = 0;

/* ── Public shared state ──────────────────────────────────────────────────── */
volatile char        rxBuffer[RX_BUFFER_SIZE]   = {0};
volatile char        payload_data[PAY_LOAD_SIZE] = {0};
volatile uint16_t    rx_index         = 0;
volatile bool        mqtt_data_ready  = false;
volatile uint32_t    mqtt_msg_count   = 0;
volatile bool        mqtt_disconnected = false;
volatile bool        urc_line_pending  = false;

/* Weak default: byte USART1 debug diabaikan. Override di main.c. */
__weak void UART_OnDebugByte(uint8_t b) { (void)b; }

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Init                                                                       */
/* ─────────────────────────────────────────────────────────────────────────── */

void UART_Init_Buffer(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    ENTER_CRITICAL();
    memset((void *)rxBuffer, 0, RX_BUFFER_SIZE);
    rx_index          = 0;
    mqtt_data_ready   = false;
    mqtt_msg_count    = 0;
    mqtt_disconnected = false;
    EXIT_CRITICAL();

    HAL_UART_Receive_IT(s_huart, &rx_byte, 1);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Buffer helpers                                                             */
/* ─────────────────────────────────────────────────────────────────────────── */

void UART_ClearBuffer(void)
{
    ENTER_CRITICAL();
    memset((void *)rxBuffer, 0, RX_BUFFER_SIZE);
    rx_index = 0;
    EXIT_CRITICAL();
}

void UART_FlushRx(uint32_t wait_ms)
{
    uint32_t  deadline    = HAL_GetTick() + wait_ms;
    uint16_t  last_index  = 0;
    uint16_t  curr_index  = 0;

    UART_ClearBuffer();

    while (HAL_GetTick() < deadline)
    {
        ENTER_CRITICAL();
        curr_index = rx_index;
        EXIT_CRITICAL();

        if (curr_index != last_index)
        {
            last_index = curr_index;
            UART_ClearBuffer();
        }
        HAL_Delay(5);
    }
    UART_ClearBuffer();
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  ISR — HAL_UART_RxCpltCallback                                             */
/* ─────────────────────────────────────────────────────────────────────────── */

/* Byte tunggal utk USART1 debug — dipakai di ISR & di-arm ulang tiap byte */
static uint8_t uart1_dbg_byte;

/* ISR RINGAN — HANYA memasukkan byte + set flag baris baru.
 *  Empat strstr() yang dulu di dalam ISR sudah dipindah ke UART_ProcessURC()
 *  yang dijalankan dari superloop. Ini yang membuat huart3 aman menerima
 *  byte terus-menerus tanpa memicu ORE / kehilangan byte. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_huart)
    {
        /* USART1 debug — byte dilempar ke hook aplikasi */
        if (huart->Instance == USART1)
        {
            UART_OnDebugByte(uart1_dbg_byte);
            HAL_UART_Receive_IT(huart, &uart1_dbg_byte, 1);
        }
        return;
    }

    if (rx_index < RX_BUFFER_SIZE - 1)
    {
        rxBuffer[rx_index++] = rx_byte;
        rxBuffer[rx_index]   = '\0';
        if (rx_byte == '\n') urc_line_pending = true;
    }
    else
    {
        rx_index    = 0;
        rxBuffer[0] = '\0';
    }

    HAL_UART_Receive_IT(s_huart, &rx_byte, 1);
}

/* Scan URC penting di context main (bukan ISR). Aman jalan berapa lama pun. */
void UART_ProcessURC(void)
{
    if (!urc_line_pending) return;
    urc_line_pending = false;

    if (strstr((const char *)rxBuffer, "+QMTRECV:") != NULL) {
        mqtt_msg_count++;
        mqtt_data_ready = true;
    }
    if (strstr((const char *)rxBuffer, "+QMTSTAT: 0,1") != NULL ||
        strstr((const char *)rxBuffer, "+QMTSTAT: 0,2") != NULL ||
        strstr((const char *)rxBuffer, "+QMTSTAT: 0,3") != NULL) {
        mqtt_disconnected = true;
    }
}

/* [PERBAIKAN] Handler error UART. Kalau ISR sempat telat dan byte baru
 * menyusul, F1 set flag ORE (Overrun Error). Kalau flag ini tidak di-clear,
 * HAL membatalkan HAL_UART_Receive_IT dan BERHENTI menerima byte SELAMANYA.
 * Handler ini clear flag (F1: baca SR lalu DR) dan re-arm RX-IT. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    volatile uint32_t sr = huart->Instance->SR;
    volatile uint32_t dr = huart->Instance->DR;
    (void)sr; (void)dr;

    if (huart == s_huart)
        HAL_UART_Receive_IT(s_huart, &rx_byte, 1);
    else if (huart->Instance == USART1)
        HAL_UART_Receive_IT(huart, &uart1_dbg_byte, 1);
}

/* Panggilan ini juga dipakai di main.c untuk mulai RX USART1 pertama kali. */
void UART_StartDebugRx(void)
{
    HAL_UART_Receive_IT(&huart1, &uart1_dbg_byte, 1);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Transmit helpers                                                           */
/* ─────────────────────────────────────────────────────────────────────────── */

void UART_SendString(const char *str)
{
    if (s_huart == NULL || str == NULL) return;
    HAL_UART_Transmit(s_huart, (uint8_t *)str, (uint16_t)strlen(str), 5000);
}

void UART_TransmitRaw(const uint8_t *data, uint16_t len)
{
    if (s_huart == NULL || data == NULL || len == 0) return;
    HAL_UART_Transmit(s_huart, (uint8_t *)data, len, 5000);
}

void UART_SendATCommand(const char *cmd_no_crlf)
{
    if (s_huart == NULL || cmd_no_crlf == NULL) return;

    UART_ClearBuffer();

    static char buf[512];
    snprintf(buf, sizeof(buf), "%s\r\n", cmd_no_crlf);
    HAL_UART_Transmit(s_huart, (uint8_t *)buf, (uint16_t)strlen(buf), 5000);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Internal: thread-safe substring search in rxBuffer                        */
/* ─────────────────────────────────────────────────────────────────────────── */

static bool buffer_contains(const char *str)
{
    bool result = false;
    ENTER_CRITICAL();
    if (rx_index > 0)
        result = (strstr((const char *)rxBuffer, str) != NULL);
    EXIT_CRITICAL();
    return result;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Watchdog refresh hook                                                      */
/* ─────────────────────────────────────────────────────────────────────────── */

__weak void UART_WatchdogRefresh(void) {}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Debug UART poll hook                                                       */
/*                                                                             */
/*  Dipanggil setiap iterasi polling di semua wait loop (WaitForOK/URC/       */
/*  Prompt/OK_Then_URC). Override di main.c untuk memanggil UID_Process()     */
/*  supaya perintah SETUID/GETUID/STATUS (dan yang akan ditambahkan) TETAP    */
/*  bisa diproses saat modem sedang di-tunggu — termasuk selama MQTT_Open     */
/*  yang bisa memakan detik. Byte-nya sendiri sudah selalu masuk ke buffer    */
/*  lewat ISR huart1; hook ini yang membuat *pemrosesannya* juga tidak macet. */
/* ─────────────────────────────────────────────────────────────────────────── */

__weak void UART_DebugPoll(void) {}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Response polling                                                           */
/* ─────────────────────────────────────────────────────────────────────────── */

/* Interval pemanggilan watchdog di dalam loop polling (ms) */
#define WDG_REFRESH_INTERVAL_MS  5000U

bool UART_WaitForOK(uint32_t timeout_ms)
{
    uint32_t start    = HAL_GetTick();
    uint32_t wdg_last = start;
    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (buffer_contains("OK"))    return true;
        if (buffer_contains("ERROR")) return false;
        HAL_Delay(5);
        UART_DebugPoll();   /* proses perintah debug (SETUID dsb) walau lagi tunggu modem */
        if ((HAL_GetTick() - wdg_last) >= WDG_REFRESH_INTERVAL_MS)
        {
            UART_WatchdogRefresh();
            wdg_last = HAL_GetTick();
        }
    }
    return false;
}

bool UART_WaitForURC(const char *urc_expected, uint32_t timeout_ms)
{
    uint32_t start    = HAL_GetTick();
    uint32_t wdg_last = start;
    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (buffer_contains(urc_expected)) return true;
        if (buffer_contains("ERROR"))      return false;
        HAL_Delay(5);
        UART_DebugPoll();   /* proses perintah debug (SETUID dsb) walau lagi tunggu modem */
        if ((HAL_GetTick() - wdg_last) >= WDG_REFRESH_INTERVAL_MS)
        {
            UART_WatchdogRefresh();
            wdg_last = HAL_GetTick();
        }
    }
    return false;
}

bool UART_WaitForPrompt(const char *prompt, uint32_t timeout_ms)
{
    uint32_t start    = HAL_GetTick();
    uint32_t wdg_last = start;
    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (buffer_contains(prompt)) return true;
        HAL_Delay(5);
        UART_DebugPoll();   /* proses perintah debug (SETUID dsb) walau lagi tunggu modem */
        if ((HAL_GetTick() - wdg_last) >= WDG_REFRESH_INTERVAL_MS)
        {
            UART_WatchdogRefresh();
            wdg_last = HAL_GetTick();
        }
    }
    return false;
}

bool UART_WaitFor_OK_Then_URC(const char *urc_expected,
                               uint32_t timeout_ok_ms,
                               uint32_t timeout_urc_ms)
{
    uint32_t start    = HAL_GetTick();
    uint32_t wdg_last = start;
    bool     ok_found = false;

    /* Phase 1: tunggu OK */
    while ((HAL_GetTick() - start) < timeout_ok_ms)
    {
        if (buffer_contains("OK"))    { ok_found = true; break; }
        if (buffer_contains("ERROR")) return false;
        HAL_Delay(5);
        UART_DebugPoll();   /* proses perintah debug (SETUID dsb) walau lagi tunggu modem */
        if ((HAL_GetTick() - wdg_last) >= WDG_REFRESH_INTERVAL_MS)
        {
            UART_WatchdogRefresh();
            wdg_last = HAL_GetTick();
        }
    }
    if (!ok_found) return false;

    /* Phase 2: tunggu URC */
    start    = HAL_GetTick();
    wdg_last = start;
    while ((HAL_GetTick() - start) < timeout_urc_ms)
    {
        if (buffer_contains(urc_expected)) return true;
        if (buffer_contains("ERROR"))      return false;
        HAL_Delay(5);
        UART_DebugPoll();   /* proses perintah debug (SETUID dsb) walau lagi tunggu modem */
        if ((HAL_GetTick() - wdg_last) >= WDG_REFRESH_INTERVAL_MS)
        {
            UART_WatchdogRefresh();
            wdg_last = HAL_GetTick();
        }
    }
    return false;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Payload extraction                                                         */
/* ─────────────────────────────────────────────────────────────────────────── */

bool Extract_Payload(void)
{
    memset((void *)payload_data, 0, sizeof(payload_data));

    char *start = strstr((const char *)rxBuffer, "\"DATA\":");
    if (start == NULL) return false;

    start += 7;
    while (*start == ' ') start++;
    if (*start != '{') return false;

    int   bracket_count = 0;
    char *ptr           = start;
    char *end           = NULL;

    while (*ptr != '\0')
    {
        if      (*ptr == '{') bracket_count++;
        else if (*ptr == '}')
        {
            bracket_count--;
            if (bracket_count == 0) { end = ptr; break; }
        }
        ptr++;
    }
    if (end == NULL) return false;

    int len = (int)(end - start) + 1;
    if (len >= (int)sizeof(payload_data))
        len = (int)sizeof(payload_data) - 1;

    strncpy((char *)payload_data, start, (size_t)len);
    payload_data[len] = '\0';
    return true;
}
