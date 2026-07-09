/*
 * modem_check.c — Pengecekan status modem Quectel EC25/EG25 saat startup.
 *
 * Meniru urutan QNavigator. Memakai modul uart.c kamu:
 *   - UART_SendATCommand()  : clear buffer + kirim "cmd\r\n"
 *   - UART_WaitForOK()      : tunggu "OK"
 *   - rxBuffer[]            : dibaca langsung untuk parsing respon
 *
 * Debug print dikirim ke huart1 (sama seperti Debug_Print_Sensor di main.c).
 */

#include "modem_check.h"
#include "uart.h"
#include "usart.h"       /* huart1 untuk debug print */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ── Helper: cetak baris debug ke huart1 ──────────────────────────────────── */
static void dbg(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 500);
}

/* ── Helper: ambil salinan rxBuffer yang aman (volatile → lokal) ──────────── */
static void snapshot_rx(char *dst, size_t dst_size)
{
    size_t i = 0;
    ENTER_CRITICAL();
    for (i = 0; i < dst_size - 1 && rxBuffer[i] != '\0'; i++)
        dst[i] = rxBuffer[i];
    dst[i] = '\0';
    EXIT_CRITICAL();
}

/* ── Helper: kirim AT command, tunggu OK, kembalikan salinan respon ───────── */
static bool at_query(const char *cmd, char *resp, size_t resp_size, uint32_t timeout_ms)
{
    UART_SendATCommand(cmd);
    bool ok = UART_WaitForOK(timeout_ms);
    if (resp && resp_size) snapshot_rx(resp, resp_size);
    return ok;
}

/* ── Helper: cari nilai setelah token (mis. "+CSQ:") lalu skip spasi ──────── */
static const char *after_token(const char *buf, const char *token)
{
    const char *p = strstr(buf, token);
    if (!p) return NULL;
    p += strlen(token);
    while (*p == ' ') p++;
    return p;
}

/* ── AT sync: kirim AT tiap 500ms, max 10x, sampai OK ─────────────────────── */
static bool modem_sync(void)
{
    for (int i = 0; i < 10; i++)
    {
        UART_SendATCommand("AT");
        if (UART_WaitForOK(1000))
        {
            dbg("[MODEM] AT sync OK\r\n");
            return true;
        }
        HAL_Delay(500);
    }
    dbg("[MODEM] AT sync GAGAL\r\n");
    return false;
}

/* ────────────────────────────────────────────────────────────────────────── */
bool Modem_SyncAndCheck(ModemStatus *st)
{
    char resp[RX_BUFFER_SIZE];
    char line[80];
    ModemStatus local = {0};
    if (st == NULL) st = &local;

    /* (1) Sync AT */
    if (!modem_sync()) return false;

    /* (2) Set format respon (seperti QNavigator) */
    at_query("ATV1",     resp, sizeof(resp), 1000);   /* verbose response       */
    at_query("ATE1",     resp, sizeof(resp), 1000);   /* echo ON (biar mirip)   */
    at_query("AT+CMEE=2",resp, sizeof(resp), 1000);   /* error verbose          */

    /* (3) Baudrate: AT+IPR? */
    if (at_query("AT+IPR?", resp, sizeof(resp), 1000))
    {
        const char *p = after_token(resp, "+IPR:");
        if (p) { st->ipr = atoi(p);
            snprintf(line, sizeof(line), "[MODEM] IPR (baud) = %d\r\n", st->ipr); dbg(line);
            if (st->ipr != 115200)
                dbg("[MODEM] ! baud bukan 115200 — cek UART firmware / AT+IPR=115200;AT&W\r\n");
        }
    }

    /* (4) Info modul: ATI */
    at_query("ATI", resp, sizeof(resp), 1000);

    /* (5) IMEI: AT+GSN */
    if (at_query("AT+GSN", resp, sizeof(resp), 1000))
    {
        /* IMEI = 15 digit; cari deretan angka pertama di respon */
        const char *p = resp;
        while (*p && (*p < '0' || *p > '9')) p++;
        int n = 0;
        while (p[n] >= '0' && p[n] <= '9' && n < 19) { st->imei[n] = p[n]; n++; }
        st->imei[n] = '\0';
        snprintf(line, sizeof(line), "[MODEM] IMEI = %s\r\n", st->imei); dbg(line);
    }

    /* (6) SIM status: AT+CPIN? */
    if (at_query("AT+CPIN?", resp, sizeof(resp), 2000))
    {
        if (strstr(resp, "READY") != NULL)
        {
            st->sim_ready = true;
            dbg("[MODEM] SIM READY\r\n");
        }
        else
        {
            dbg("[MODEM] SIM TIDAK READY\r\n");
        }
    }

    /* (7) IMSI: AT+CIMI */
    if (at_query("AT+CIMI", resp, sizeof(resp), 1000))
    {
        const char *p = resp;
        while (*p && (*p < '0' || *p > '9')) p++;
        int n = 0;
        while (p[n] >= '0' && p[n] <= '9' && n < 19) { st->imsi[n] = p[n]; n++; }
        st->imsi[n] = '\0';
        snprintf(line, sizeof(line), "[MODEM] IMSI = %s\r\n", st->imsi); dbg(line);
    }

    /* (8) ICCID: AT+QCCID */
    if (at_query("AT+QCCID", resp, sizeof(resp), 1000))
    {
        const char *p = after_token(resp, "+QCCID:");
        if (p)
        {
            int n = 0;
            while (((p[n] >= '0' && p[n] <= '9') || (p[n]>='A'&&p[n]<='F')) && n < 23)
            { st->iccid[n] = p[n]; n++; }
            st->iccid[n] = '\0';
            snprintf(line, sizeof(line), "[MODEM] ICCID = %s\r\n", st->iccid); dbg(line);
        }
    }

    /* (9) Sinyal: AT+CSQ */
    if (at_query("AT+CSQ", resp, sizeof(resp), 1000))
    {
        const char *p = after_token(resp, "+CSQ:");
        if (p) { st->csq = atoi(p);
            snprintf(line, sizeof(line), "[MODEM] CSQ (RSSI) = %d\r\n", st->csq); dbg(line);
            if (st->csq == 99) dbg("[MODEM] ! sinyal tidak terukur (99)\r\n");
        }
    }

    /* (10) Registrasi jaringan: CREG / CGREG / CEREG
     *      Format: +CREG: <n>,<stat>. stat 1=home, 5=roaming → terdaftar. */
    st->registered = false;
    const char *regcmds[3] = { "AT+CREG?", "AT+CGREG?", "AT+CEREG?" };
    const char *regtok[3]  = { "+CREG:",   "+CGREG:",   "+CEREG:"   };
    for (int i = 0; i < 3; i++)
    {
        if (at_query(regcmds[i], resp, sizeof(resp), 1000))
        {
            const char *p = after_token(resp, regtok[i]);
            if (p)
            {
                /* lewati <n>, ambil <stat> setelah koma */
                const char *comma = strchr(p, ',');
                int stat = comma ? atoi(comma + 1) : -1;
                snprintf(line, sizeof(line), "[MODEM] %s stat = %d\r\n", regtok[i], stat); dbg(line);
                if (stat == 1 || stat == 5) st->registered = true;
            }
        }
    }

    /* (11) Operator: AT+COPS? */
    if (at_query("AT+COPS?", resp, sizeof(resp), 1000))
    {
        const char *p = strchr(resp, '"');
        if (p)
        {
            p++;
            int n = 0;
            while (p[n] && p[n] != '"' && n < 39) { st->operator_name[n] = p[n]; n++; }
            st->operator_name[n] = '\0';
            snprintf(line, sizeof(line), "[MODEM] Operator = %s\r\n", st->operator_name); dbg(line);
        }
    }

    /* Ringkasan */
    if (st->sim_ready && st->registered)
    {
        dbg("[MODEM] STATUS OK: SIM ready & terdaftar jaringan\r\n");
        return true;
    }
    dbg("[MODEM] STATUS BELUM SIAP (SIM/registrasi)\r\n");
    return false;
}

/* ── Set baud 115200 permanen ─────────────────────────────────────────────── */
bool Modem_FixBaudrate115200(void)
{
    UART_SendATCommand("AT+IPR=115200");
    if (!UART_WaitForOK(1000)) return false;
    UART_SendATCommand("AT&W");                 /* simpan ke NVM */
    return UART_WaitForOK(1000);
}
