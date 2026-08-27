/*
 * uid_config.c — Kelola UID device + perintah debug via UART1.
 *
 * Format perintah yang didukung:
 *
 *   SETUID:KWH01310000099          Set UID device (tidak berubah dari lama)
 *
 *   KWH,GETUID                     Cetak UID aktif
 *   KWH,STATUS                     Cetak ringkasan status (Debug_PrintStatus)
 *   KWH,RSTWH                      resetWH() + timpa FRAM WH=0
 *   KWH,PUB                        Publish_Full_Payload(&pwr_val, WH)  [deferred]
 *   KWH,RESTART                    NVIC_SystemReset()
 *   KWH,RELAY,0                    Relay_Set(0)  (OFF)
 *   KWH,RELAY,1                    Relay_Set(1)  (ON)
 *
 *   MQTT,CLOSE                     MQTT_Close()               [deferred]
 *   MQTT,DSC                       MQTT_Disconnect()          [deferred]
 *   MQTT,OPEN                      while(MQTT_Open()!=OK) max 3x  [deferred]
 *   MQTT,CONN                      while(MQTT_Connect()!=OK) max 3x  [deferred]
 *   MQTT,SUBS                      while(MQTT_Subscribe()!=OK) max 3x  [deferred]
 *
 * Aturan format:
 *   - Case-insensitive:   kwh,getuid  ==  KWH,GETUID  ==  Kwh,GetUid
 *   - Toleran spasi:      "KWH , GETUID"  == "KWH,GETUID"  == "  KWH,GETUID  "
 *   - Prefix strict:      tanpa "KWH," atau "MQTT," (kecuali SETUID) → error
 *
 * "Deferred" berarti UID_Process cuma set flag dbg_pending. Perintah aslinya
 * dieksekusi di superloop main.c lewat Debug_HandleDeferred() — ini "opsi aman"
 * supaya perintah yang menyentuh UART3/MQTT tidak merusak rxBuffer transaksi
 * yang sedang menunggu URC.
 */

#include "uid_config.h"
#include "fram.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#define UID_MAGIC       0xA5
#define UID_ADDR_MAGIC  addr_sn
#define UID_ADDR_STR    (addr_sn + 1)

/* UID aktif */
char device_uid[UID_MAXLEN] = UID_DEFAULT;

/* Flag pending action untuk superloop */
volatile DbgPendingAction dbg_pending = DBG_ACT_NONE;

/* Buffer penerima perintah dari huart1 (ISR menulis, superloop membaca) */
static char    cmd_buf[64];       /* diperbesar dari 48 untuk MQTT,XXX + margin */
static uint8_t cmd_idx = 0;
static volatile bool cmd_ready = false;

/* ── Helper cetak ke huart1 ───────────────────────────────────────────────── */
static void uid_dbg(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 300);
}

/* ── Validasi karakter UID (alfanumerik) ──────────────────────────────────── */
static bool uid_char_ok(char c)
{
    return (c >= '0' && c <= '9') ||
           (c >= 'A' && c <= 'Z') ||
           (c >= 'a' && c <= 'z');
}

/* ── Baca UID dari FRAM ───────────────────────────────────────────────────── */
void UID_Load(void)
{
    uint8_t magic = ReadData_FRAM(UID_ADDR_MAGIC);
    if (magic != UID_MAGIC)
    {
        strncpy(device_uid, UID_DEFAULT, UID_MAXLEN - 1);
        device_uid[UID_MAXLEN - 1] = '\0';
        return;
    }

    for (int i = 0; i < UID_MAXLEN - 1; i++)
    {
        uint8_t c = ReadData_FRAM(UID_ADDR_STR + i);
        if (c == '\0' || c == 0xFF) { device_uid[i] = '\0'; break; }
        device_uid[i] = (char)c;
        device_uid[i + 1] = '\0';
    }

    if (device_uid[0] == '\0')
    {
        strncpy(device_uid, UID_DEFAULT, UID_MAXLEN - 1);
        device_uid[UID_MAXLEN - 1] = '\0';
    }
}

/* ── Simpan UID ke FRAM ───────────────────────────────────────────────────── */
bool UID_Save(const char *new_uid)
{
    if (new_uid == NULL) return false;
    size_t len = strlen(new_uid);
    if (len == 0 || len >= UID_MAXLEN) return false;

    WriteChar_FRAM(UID_ADDR_STR, (char*)new_uid);
    WriteData_FRAM(UID_ADDR_MAGIC, UID_MAGIC);

    strncpy(device_uid, new_uid, UID_MAXLEN - 1);
    device_uid[UID_MAXLEN - 1] = '\0';
    return true;
}

/* ── Terima byte dari ISR (huart1) ────────────────────────────────────────── */
void UID_FeedByte(uint8_t b)
{
    if (cmd_ready) return;   /* masih ada perintah belum diproses */

    if (b == '\r' || b == '\n')
    {
        if (cmd_idx > 0)
        {
            cmd_buf[cmd_idx] = '\0';
            cmd_ready = true;
        }
    }
    else if (cmd_idx < sizeof(cmd_buf) - 1)
    {
        cmd_buf[cmd_idx++] = (char)b;
    }
    else
    {
        cmd_idx = 0;             /* overflow → buang, mulai ulang */
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Parser helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Skip whitespace di awal & akhir, edit string in-place. Return pointer ke
 * karakter non-space pertama. */
static char *str_trim(char *s)
{
    while (*s == ' ' || *s == '\t') s++;
    char *end = s + strlen(s);
    while (end > s && (*(end - 1) == ' ' || *(end - 1) == '\t' ||
                       *(end - 1) == '\r' || *(end - 1) == '\n'))
        end--;
    *end = '\0';
    return s;
}

/* Case-insensitive strcmp — kembalikan 0 kalau sama. */
static int str_iequal(const char *a, const char *b)
{
    while (*a && *b)
    {
        char ca = (char)toupper((unsigned char)*a);
        char cb = (char)toupper((unsigned char)*b);
        if (ca != cb) return 1;
        a++; b++;
    }
    return (*a == *b) ? 0 : 1;
}

/* Split by ',' — edit str in-place, isi tokens[] dengan pointer ke tiap token
 * yang sudah di-trim. Return jumlah token. */
static int split_comma(char *str, char *tokens[], int max_tokens)
{
    int count = 0;
    char *tok_start = str;

    while (*tok_start != '\0' && count < max_tokens)
    {
        /* skip leading whitespace */
        while (*tok_start == ' ' || *tok_start == '\t') tok_start++;

        /* find next comma or end */
        char *p = tok_start;
        while (*p != '\0' && *p != ',') p++;

        char *end_of_token = p;   /* exclusive */
        bool  had_comma    = (*p == ',');
        if (had_comma) { *p = '\0'; p++; }

        /* trim trailing whitespace of token */
        while (end_of_token > tok_start &&
               (*(end_of_token - 1) == ' ' || *(end_of_token - 1) == '\t'))
            end_of_token--;
        *end_of_token = '\0';

        tokens[count++] = tok_start;

        if (!had_comma) break;
        tok_start = p;
    }
    return count;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Eksekusi handler cepat (fast/safe from any context)
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Handler untuk kelompok "KWH,..." — return true kalau perintah dikenali
 * (walaupun eksekusinya deferred). */
static bool exec_kwh_cmd(char *tokens[], int ntok)
{
    /* tokens[0] sudah pasti "KWH" (case-normalized ke UPPER). Butuh minimal
     * satu subcommand. */
    if (ntok < 2) return false;
    const char *sub = tokens[1];

    if (str_iequal(sub, "GETUID") == 0)
    {
        uid_dbg("[KWH] UID = ");
        uid_dbg(device_uid);
        uid_dbg("\r\n");
        return true;
    }
    if (str_iequal(sub, "STATUS") == 0)
    {
        Debug_PrintStatus();
        return true;
    }
    if (str_iequal(sub, "RSTWH") == 0)
    {
        Debug_ResetWH_Full();   /* di main.c: resetWH() + tulis 0 ke FRAM */
        uid_dbg("[KWH] WH direset ke 0 (RAM + FRAM)\r\n");
        return true;
    }
    if (str_iequal(sub, "PUB") == 0)
    {
        dbg_pending = DBG_ACT_PUB_FULL;
        uid_dbg("[KWH] PUB antre — akan dikirim di superloop\r\n");
        return true;
    }
    if (str_iequal(sub, "RESTART") == 0)
    {
        uid_dbg("[KWH] Restart device...\r\n");
        HAL_Delay(200);
        NVIC_SystemReset();
        while (1) {}
    }
    if (str_iequal(sub, "RELAY") == 0)
    {
        if (ntok < 3)
        {
            uid_dbg("[KWH] RELAY butuh parameter: 0 atau 1\r\n");
            return true;   /* dikenali, meski invalid parameter */
        }
        const char *val = tokens[2];
        if (val[0] == '0' && val[1] == '\0')
        {
            Debug_SetRelay(0);
            uid_dbg("[KWH] RELAY OFF\r\n");
            return true;
        }
        if (val[0] == '1' && val[1] == '\0')
        {
            Debug_SetRelay(1);
            uid_dbg("[KWH] RELAY ON\r\n");
            return true;
        }
        uid_dbg("[KWH] RELAY parameter harus 0 atau 1\r\n");
        return true;
    }
    return false;
}

/* Handler untuk kelompok "MQTT,..." — semua deferred karena sentuh UART3. */
static bool exec_mqtt_cmd(char *tokens[], int ntok)
{
    if (ntok < 2) return false;
    const char *sub = tokens[1];

    if (str_iequal(sub, "CLOSE") == 0)
    {
        dbg_pending = DBG_ACT_MQTT_CLOSE;
        uid_dbg("[MQTT] CLOSE antre\r\n");
        return true;
    }
    if (str_iequal(sub, "DSC") == 0)
    {
        dbg_pending = DBG_ACT_MQTT_DSC;
        uid_dbg("[MQTT] DSC antre\r\n");
        return true;
    }
    if (str_iequal(sub, "OPEN") == 0)
    {
        dbg_pending = DBG_ACT_MQTT_OPEN;
        uid_dbg("[MQTT] OPEN antre (max 3x retry)\r\n");
        return true;
    }
    if (str_iequal(sub, "CONN") == 0)
    {
        dbg_pending = DBG_ACT_MQTT_CONN;
        uid_dbg("[MQTT] CONN antre (max 3x retry)\r\n");
        return true;
    }
    if (str_iequal(sub, "SUBS") == 0)
    {
        dbg_pending = DBG_ACT_MQTT_SUBS;
        uid_dbg("[MQTT] SUBS antre (max 3x retry)\r\n");
        return true;
    }
    return false;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Main parser
 * ═══════════════════════════════════════════════════════════════════════════ */

void UID_Process(void)
{
    if (!cmd_ready) return;

    /* SETUID:xxx tetap seperti lama, tidak berubah format. */
    if (strncmp(cmd_buf, "SETUID:", 7) == 0)
    {
        const char *val = cmd_buf + 7;
        size_t len = strlen(val);
        bool ok = (len > 0 && len < UID_MAXLEN);
        for (size_t i = 0; i < len && ok; i++)
            if (!uid_char_ok(val[i])) ok = false;

        if (ok)
        {
            if (UID_Save(val))
            {
                uid_dbg("[UID] tersimpan: ");
                uid_dbg(val);
                uid_dbg("\r\n[UID] restart device...\r\n");
                HAL_Delay(200);
                NVIC_SystemReset();
                while (1) {}
            }
            uid_dbg("[UID] GAGAL simpan ke FRAM\r\n");
        }
        else
        {
            uid_dbg("[UID] format salah (hanya A-Z a-z 0-9, maks 23 char)\r\n");
        }
        goto done;
    }

    /* Untuk perintah lain (KWH,... dan MQTT,...) — split by ',' lalu route. */
    char work[64];
    strncpy(work, cmd_buf, sizeof(work) - 1);
    work[sizeof(work) - 1] = '\0';

    char *tokens[4] = {0};
    int   ntok = split_comma(str_trim(work), tokens, 4);

    if (ntok < 1 || tokens[0][0] == '\0')
    {
        uid_dbg("[DBG] perintah kosong\r\n");
        goto done;
    }

    /* Route berdasarkan prefix — case insensitive. */
    if (str_iequal(tokens[0], "KWH") == 0)
    {
        if (!exec_kwh_cmd(tokens, ntok))
        {
            uid_dbg("[KWH] subcommand tidak dikenali\r\n");
            uid_dbg("[KWH] daftar: GETUID | STATUS | RSTWH | PUB | RESTART | RELAY,0|1\r\n");
        }
        goto done;
    }
    if (str_iequal(tokens[0], "MQTT") == 0)
    {
        if (!exec_mqtt_cmd(tokens, ntok))
        {
            uid_dbg("[MQTT] subcommand tidak dikenali\r\n");
            uid_dbg("[MQTT] daftar: CLOSE | DSC | OPEN | CONN | SUBS\r\n");
        }
        goto done;
    }

    /* Prefix tidak dikenal — strict: gagal. */
    uid_dbg("[DBG] format salah — gunakan prefix KWH,... atau MQTT,... (atau SETUID:xxx)\r\n");

done:
    cmd_idx = 0;
    cmd_ready = false;
}
