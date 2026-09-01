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
volatile uint32_t    rx_overflow_count = 0;   /* [FIX #1] */

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

/* ── [FIX] Scan +QMTSTAT SEBELUM rxBuffer di-wipe ─────────────────────────────
 *
 * BUG YANG DIPERBAIKI: UART_ProcessURC() (dipanggil sekali per iterasi
 * superloop) adalah SATU-SATUNYA tempat yang men-scan rxBuffer untuk
 * "+QMTSTAT: 0,1/2/3" dan set mqtt_disconnected=true. Tapi UART_ClearBuffer()
 * dipanggil dari BANYAK tempat lain (paling sering: UART_SendATCommand(),
 * yaitu di AWAL setiap AT command baru) — dan itu bisa terjadi KAPAN SAJA,
 * termasuk di tengah rangkaian beberapa AT command berturut-turut (mis.
 * MQTT_Reconnect() yang kirim 5-6 command tanpa pernah balik ke superloop
 * di antaranya). Kalau modem kebetulan mengirim URC "+QMTSTAT: 0,x" persis
 * di jendela waktu itu, teksnya masuk ke rxBuffer, TAPI keburu di-wipe oleh
 * UART_ClearBuffer() dari command berikutnya SEBELUM UART_ProcessURC() di
 * superloop sempat membacanya. Hasilnya: mqtt_disconnected TIDAK PERNAH
 * ke-set walau modem sudah memberi tahu koneksi putus — status tetap
 * tercatat "TERHUBUNG" padahal publish/subscribe sebenarnya sudah gagal.
 *
 * PERBAIKAN: setiap kali UART_ClearBuffer() dipanggil, scan dulu isi
 * rxBuffer untuk "+QMTSTAT: 0,1/2/3" SEBELUM benar-benar di-wipe. Dengan
 * begitu, URC itu tidak pernah "kabur" lagi — terdeteksi tepat di titik
 * manapun yang mencoba menghapusnya, bukan hanya lewat jalur periodik
 * UART_ProcessURC(). (+QMTRECV sengaja TIDAK di-scan di sini — payload-nya
 * perlu tetap utuh untuk MQTT_ProcessIncoming(), bukan sekadar flag boolean,
 * jadi tetap ditangani lewat UART_ProcessURC() seperti semula.)         */
static void scan_qmtstat_before_clear(void)
{
    if (rx_index == 0) return;   /* buffer memang sudah kosong */

    if (strstr((const char *)rxBuffer, "+QMTSTAT: 0,1") != NULL ||
        strstr((const char *)rxBuffer, "+QMTSTAT: 0,2") != NULL ||
        strstr((const char *)rxBuffer, "+QMTSTAT: 0,3") != NULL)
    {
        mqtt_disconnected = true;
    }
}

void UART_ClearBuffer(void)
{
    ENTER_CRITICAL();
    scan_qmtstat_before_clear();
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

/* ── [RAWMON] Ring buffer byte mentah huart3 ─────────────────────────────────
 * Terpisah total dari rxBuffer supaya TIDAK mengganggu parsing AT/MQTT yang
 * sudah ada. Ditulis di ISR (murah: index + copy byte saja), dikuras dari
 * superloop lewat UART3_RawMon_Poll(). Ukuran kelipatan 2 → wrap pakai '&'. */
#define UART3_RAWMON_RING_SIZE   256U   /* HARUS kelipatan 2 */
#define UART3_RAWMON_RING_MASK   (UART3_RAWMON_RING_SIZE - 1U)
#define UART3_RAWMON_IDLE_MS     3000U  /* anggap "diam" kalau tak ada byte selama ini */

static volatile uint8_t  uart3_raw_ring[UART3_RAWMON_RING_SIZE];
static volatile uint16_t uart3_raw_head = 0;   /* ditulis ISR   */
static volatile uint16_t uart3_raw_tail = 0;   /* ditulis Poll() */

volatile bool     uart3_rawmon_enabled   = false;
volatile uint32_t uart3_raw_rx_count     = 0;
volatile uint32_t uart3_raw_dropped      = 0;
volatile uint32_t uart3_raw_last_rx_tick = 0;

/* true selama periode "diam" sudah dilaporkan, supaya tidak spam tiap poll */
static bool uart3_raw_idle_reported = false;

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

    /* [RAWMON] Tap byte huart3 mentah — sebelum/terpisah dari parsing rxBuffer
     * di bawah. Sangat murah (tak ada strstr/Transmit), aman di ISR. */
    if (uart3_rawmon_enabled)
    {
        uint16_t next_head = (uint16_t)((uart3_raw_head + 1U) & UART3_RAWMON_RING_MASK);
        if (next_head != uart3_raw_tail)
        {
            uart3_raw_ring[uart3_raw_head] = rx_byte;
            uart3_raw_head = next_head;
        }
        else
        {
            uart3_raw_dropped++;   /* ring RAWMON penuh — TIDAK mempengaruhi rxBuffer di bawah */
        }
        uart3_raw_rx_count++;
    }
    uart3_raw_last_rx_tick = HAL_GetTick();

    if (rx_index >= RX_BUFFER_SIZE - 1)
    {
        /* [FIX #1] Buffer penuh TANPA newline — sebelumnya di sini seluruh
         * rxBuffer di-wipe (rx_index=0), jadi kalau lagi di tengah menunggu
         * OK/URC/payload besar, SEMUA konteks yang sudah masuk ikut hilang.
         * Sekarang: geser separuh TERAKHIR ke depan, buang separuh TERLAMA
         * saja — data yang baru saja masuk tetap ada, cuma histori lama yang
         * hilang. Dipadukan dengan RX_BUFFER_SIZE=1024 supaya kejadian ini
         * sendiri jadi jarang. */
        uint16_t shift = RX_BUFFER_SIZE / 2;
        memmove((void *)rxBuffer, (const void *)&rxBuffer[shift],
                (size_t)(RX_BUFFER_SIZE - shift));
        rx_index = (uint16_t)(RX_BUFFER_SIZE - shift);
        rx_overflow_count++;
    }

    rxBuffer[rx_index++] = rx_byte;
    rxBuffer[rx_index]   = '\0';
    if (rx_byte == '\n') urc_line_pending = true;

    HAL_UART_Receive_IT(s_huart, &rx_byte, 1);
}

/* Scan URC penting di context main (bukan ISR). Aman jalan berapa lama pun. */
void UART_ProcessURC(void)
{
    if (!urc_line_pending) return;
    urc_line_pending = false;

    /* [FIX] Baca rxBuffer di bawah ENTER_CRITICAL — sebelumnya fungsi ini
     * strstr() langsung ke rxBuffer tanpa lock, padahal ISR huart3 bisa saja
     * sedang menulis byte baru di saat bersamaan (rxBuffer volatile, ditulis
     * per-byte). Kecil kemungkinannya menyebabkan crash, tapi tetap bisa
     * membuat strstr() membaca string yang "setengah jadi" tepat di ujungnya
     * dan gagal mengenali +QMTSTAT/+QMTRECV pada saat kritis. buffer_contains()
     * di file ini sudah benar memakai lock — fungsi ini disamakan. */
    ENTER_CRITICAL();
    bool got_recv   = (strstr((const char *)rxBuffer, "+QMTRECV:") != NULL);
    bool disc_1     = (strstr((const char *)rxBuffer, "+QMTSTAT: 0,1") != NULL);
    bool disc_2     = (strstr((const char *)rxBuffer, "+QMTSTAT: 0,2") != NULL);
    bool disc_3     = (strstr((const char *)rxBuffer, "+QMTSTAT: 0,3") != NULL);
    EXIT_CRITICAL();

    if (got_recv) {
        mqtt_msg_count++;
        mqtt_data_ready = true;
    }
    if (disc_1 || disc_2 || disc_3) {
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

    /* [FIX #2] Sebelumnya fungsi ini memindai rxBuffer volatile LANGSUNG
     * tanpa critical section, padahal ISR huart3 bisa menulis byte baru +
     * menggeser rx_index kapan saja di tengah strstr()/strncpy() di bawah —
     * berisiko baca "robek" (setengah data lama, setengah baru). Sekarang
     * disalin dulu ke buffer lokal di dalam critical section (pola sama
     * seperti MQTT_ProcessIncoming()/mqtt_read_time() di mqtt.c), baru semua
     * parsing jalan di atas salinan yang stabil. */
    static char local[RX_BUFFER_SIZE];
    ENTER_CRITICAL();
    size_t n = strnlen((const char *)rxBuffer, RX_BUFFER_SIZE - 1);
    memcpy(local, (const char *)rxBuffer, n);
    EXIT_CRITICAL();
    local[n] = '\0';

    char *start = strstr(local, "\"DATA\":");
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

/* ─────────────────────────────────────────────────────────────────────────── */
/*  [RAWMON] Monitor byte mentah UART3 → dicetak ke UART1 debug               */
/* ─────────────────────────────────────────────────────────────────────────── */

/* [PENTING] RAWMON HARUS selalu keluar lewat huart1 (debug), BUKAN lewat
 * UART_SendString()/s_huart — s_huart menunjuk ke huart3 (modem)! Kalau
 * RAWMON ikut memakai UART_SendString(), teks monitor malah terkirim ke
 * modem, bukan ke terminal debug Anda. Makanya dipakai HAL_UART_Transmit
 * langsung ke &huart1 di sini, persis seperti P()/Pf() di main.c. */
static void rawmon_dbg_print(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, (uint16_t)strlen(str), 1000);
}

void UART3_RawMon_Enable(bool enable)
{
    ENTER_CRITICAL();
    uart3_raw_head       = 0;
    uart3_raw_tail       = 0;
    uart3_raw_rx_count   = 0;
    uart3_raw_dropped    = 0;
    uart3_rawmon_enabled = enable;
    uart3_raw_last_rx_tick = HAL_GetTick();
    EXIT_CRITICAL();
    uart3_raw_idle_reported = false;

    if (enable)
        rawmon_dbg_print("\r\n[RAWMON] ON — mencetak byte mentah UART3 ke sini\r\n");
    else
        rawmon_dbg_print("\r\n[RAWMON] OFF\r\n");
}

/* Cetak satu byte dalam bentuk aman-tampil: karakter cetak apa adanya,
 * '\r'/'\n' diberi penanda supaya baris tidak berantakan, byte lain (termasuk
 * byte rusak/noise) dicetak sebagai <0xHH> supaya kelihatan jelas itu bukan
 * teks AT/JSON normal. */
static void rawmon_put_byte(uint8_t b)
{
    char out[8];
    int  n;

    if (b == '\r') { rawmon_dbg_print("\\r"); return; }
    if (b == '\n') { rawmon_dbg_print("\\n\r\n"); return; }

    if (b >= 0x20 && b <= 0x7E)
    {
        out[0] = (char)b;
        out[1] = '\0';
        rawmon_dbg_print(out);
    }
    else
    {
        n = snprintf(out, sizeof(out), "<%02X>", b);
        (void)n;
        rawmon_dbg_print(out);
    }
}

void UART3_RawMon_Poll(void)
{
    if (!uart3_rawmon_enabled)
        return;

    /* Kuras ring buffer. Dibatasi per panggilan supaya polling lain
     * (UART_ProcessURC, ProcessCmd, dst.) di superloop tetap responsif kalau
     * traffic UART3 sedang deras. */
    uint16_t drained = 0;
    while (drained < 128U)
    {
        uint16_t tail;
        uint16_t head;
        ENTER_CRITICAL();
        tail = uart3_raw_tail;
        head = uart3_raw_head;
        EXIT_CRITICAL();

        if (tail == head) break;   /* ring buffer kosong */

        uint8_t b = uart3_raw_ring[tail];
        ENTER_CRITICAL();
        uart3_raw_tail = (uint16_t)((tail + 1U) & UART3_RAWMON_RING_MASK);
        EXIT_CRITICAL();

        rawmon_put_byte(b);
        drained++;
        uart3_raw_idle_reported = false;
    }

    /* Laporkan kalau ring buffer RAWMON sendiri pernah overflow (bukan
     * rxBuffer utama) — artinya sebagian byte MENTAH tidak sempat tercetak,
     * meski byte itu tetap masuk normal ke rxBuffer/parsing modem. */
    static uint32_t last_reported_drop = 0;
    uint32_t dropped_now = uart3_raw_dropped;
    if (dropped_now < last_reported_drop)
        last_reported_drop = 0;   /* counter direset oleh UART3_RawMon_Enable() */
    if (dropped_now != last_reported_drop)
    {
        char msg[96];
        snprintf(msg, sizeof(msg),
                 "\r\n[RAWMON] !! %lu byte tidak sempat dicetak (ring RAWMON penuh)\r\n",
                 (unsigned long)(dropped_now - last_reported_drop));
        rawmon_dbg_print(msg);
        last_reported_drop = dropped_now;
    }

    /* Laporkan kalau UART3 "diam" — tidak ada byte masuk sama sekali —
     * supaya kelihatan kalau memang tidak ada data yang masuk (bukan cuma
     * tidak tercetak). Dilaporkan sekali per periode diam, tidak spam. */
    uint32_t idle_ms = HAL_GetTick() - uart3_raw_last_rx_tick;
    if (idle_ms >= UART3_RAWMON_IDLE_MS && !uart3_raw_idle_reported)
    {
        char msg[96];
        snprintf(msg, sizeof(msg),
                 "\r\n[RAWMON] tidak ada byte masuk dari UART3 selama %lu ms\r\n",
                 (unsigned long)idle_ms);
        rawmon_dbg_print(msg);
        uart3_raw_idle_reported = true;
    }
}
