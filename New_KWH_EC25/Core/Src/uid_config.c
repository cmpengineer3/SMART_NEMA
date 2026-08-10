/*
 * uid_config.c — Kelola UID device via UART debug (huart1) + FRAM.
 *
 * Format perintah:  SETUID:KWH01310000099<Enter>
 *
 * Penyimpanan FRAM:
 *   addr_sn      : magic byte (0xA5) penanda UID valid
 *   addr_sn + 1  : string UID (null-terminated)
 */

#include "uid_config.h"
#include "fram.h"
#include "usart.h"       /* huart1 untuk echo/feedback */
#include <string.h>
#include <stdio.h>

#define UID_MAGIC       0xA5
#define UID_ADDR_MAGIC  addr_sn          /* 50 */
#define UID_ADDR_STR    (addr_sn + 1)    /* 51 */

/* UID aktif */
char device_uid[UID_MAXLEN] = UID_DEFAULT;

/* Buffer penerima perintah dari huart1 */
static char    cmd_buf[48];
static uint8_t cmd_idx = 0;
static volatile bool cmd_ready = false;   /* di-set di ISR, dibaca di loop */

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
        /* Belum pernah diisi → pakai default */
        strncpy(device_uid, UID_DEFAULT, UID_MAXLEN - 1);
        device_uid[UID_MAXLEN - 1] = '\0';
        return;
    }

    /* Baca string UID byte per byte sampai NUL atau batas panjang */
    for (int i = 0; i < UID_MAXLEN - 1; i++)
    {
        uint8_t c = ReadData_FRAM(UID_ADDR_STR + i);
        if (c == '\0' || c == 0xFF) { device_uid[i] = '\0'; break; }
        device_uid[i] = (char)c;
        device_uid[i + 1] = '\0';
    }

    /* Kalau hasil baca kosong/aneh, jatuhkan ke default */
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

    /* Tulis string (WriteChar_FRAM sudah termasuk NUL) */
    WriteChar_FRAM(UID_ADDR_STR, (char*)new_uid);

    /* Tulis magic byte penanda valid */
    WriteData_FRAM(UID_ADDR_MAGIC, UID_MAGIC);

    /* Update salinan di RAM */
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
            cmd_ready = true;    /* baris lengkap, siap diproses di loop */
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

/* ── Proses perintah di loop utama ────────────────────────────────────────── */
void UID_Process(void)
{
    if (!cmd_ready) return;

    /* Cek prefix "SETUID:" */
    if (strncmp(cmd_buf, "SETUID:", 7) == 0)
    {
        const char *val = cmd_buf + 7;

        /* Validasi panjang & karakter */
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
                NVIC_SystemReset();      /* restart → topic tersusun ulang */
                while (1) {}
            }
            else
            {
                uid_dbg("[UID] GAGAL simpan ke FRAM\r\n");
            }
        }
        else
        {
            uid_dbg("[UID] format salah (hanya A-Z a-z 0-9, maks 23 char)\r\n");
        }
    }
    /* GETUID → tampilkan UID aktif */
    else if (strcmp(cmd_buf, "GETUID") == 0)
    {
        uid_dbg("[UID] device_uid saat ini: ");
        uid_dbg(device_uid);
        uid_dbg("\r\n");
    }
    /* STATUS → tampilkan ringkasan koneksi (diimplementasi di main.c) */
    else if (strcmp(cmd_buf, "STATUS") == 0)
    {
        Debug_PrintStatus();
    }
    else
    {
        uid_dbg("[UID] perintah tidak dikenali (SETUID:xxx | GETUID | STATUS)\r\n");
    }

    /* Reset buffer untuk perintah berikutnya */
    cmd_idx = 0;
    cmd_ready = false;
}
