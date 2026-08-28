/*
 * modem_check.c
 *
 *  Created on: Aug 28, 2026
 *      Author: firza
 */


#include "modem_check.h"
#include "uart.h"
#include "main.h"
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

static bool at_query(const char *cmd, char *resp, size_t resp_size, uint32_t timeout_ms)
{
    UART_SendATCommand(cmd);
    bool ok = UART_WaitForOK(timeout_ms);
    if (resp && resp_size) snapshot_rx(resp, resp_size);
    UART_WatchdogRefresh();
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
            UART_WatchdogRefresh();
            return true;
        }
        UART_WatchdogRefresh();   /* kick tiap percobaan, bukan cuma tiap 5 detik */
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
    at_query("ATV1",     resp, sizeof(resp), 1000);
    at_query("ATE1",     resp, sizeof(resp), 1000);
    at_query("AT+CMEE=2",resp, sizeof(resp), 1000);

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
        const char *p = resp;
        while (*p && (*p < '0' || *p > '9')) p++;
        int n = 0;
        while (p[n] >= '0' && p[n] <= '9' && n < 19) { st->imei[n] = p[n]; n++; }
        st->imei[n] = '\0';
        snprintf(line, sizeof(line), "[MODEM] IMEI = %s\r\n", st->imei); dbg(line);
    }


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


static void uart3_reinit(uint32_t baud)
{
    huart3.Init.BaudRate     = baud;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
    HAL_Delay(50);
    UART_Init_Buffer(&huart3);
}

/* Coba AT sync <attempts> kali dengan tunggu OK 1 detik per percobaan.
 * return true kalau ada satu percobaan yang dapat "OK". */
static bool try_at_sync(int attempts)
{
    for (int i = 0; i < attempts; i++)
    {
        UART_SendATCommand("AT");
        if (UART_WaitForOK(1000))
        {
            UART_WatchdogRefresh();
            return true;
        }
        UART_WatchdogRefresh();
        HAL_Delay(300);
    }
    return false;
}

bool Modem_AutoBaudrate(uint32_t target_baud)
{
    char line[80];

    dbg("[MODEM] AutoBaud: coba AT sync di baud saat ini...\r\n");
    if (try_at_sync(3))
    {
        snprintf(line, sizeof(line), "[MODEM] AutoBaud: OK di %lu bps (tidak perlu koreksi)\r\n",
                 (unsigned long)target_baud);
        dbg(line);
        return true;
    }

    uint32_t alt_baud = (target_baud == 9600) ? 115200u : 9600u;
    snprintf(line, sizeof(line), "[MODEM] AutoBaud: gagal di %lu, coba %lu...\r\n",
             (unsigned long)target_baud, (unsigned long)alt_baud);
    dbg(line);

    uart3_reinit(alt_baud);
    UART_WatchdogRefresh();

    if (!try_at_sync(3))
    {

        dbg("[MODEM] AutoBaud: gagal di kedua baud (modem tidak merespon)\r\n");
        uart3_reinit(target_baud);
        UART_WatchdogRefresh();
        return false;
    }

    snprintf(line, sizeof(line), "[MODEM] AutoBaud: sync OK di %lu, set modem ke %lu...\r\n",
             (unsigned long)alt_baud, (unsigned long)target_baud);
    dbg(line);

    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+IPR=%lu", (unsigned long)target_baud);
    UART_SendATCommand(cmd);

    (void)UART_WaitForOK(1500);
    UART_WatchdogRefresh();


    uart3_reinit(target_baud);
    HAL_Delay(200);
    UART_WatchdogRefresh();


    if (!try_at_sync(3))
    {
        dbg("[MODEM] AutoBaud: verifikasi di baud baru GAGAL — reset & coba lagi\r\n");
        HAL_Delay(200);
        NVIC_SystemReset();
        while (1) {}   /* not reached */
    }

    UART_SendATCommand("AT&W");
    (void)UART_WaitForOK(1500);
    UART_WatchdogRefresh();

    dbg("[MODEM] AutoBaud: baud modem tersimpan permanen di NVM. Reset MCU...\r\n");
    HAL_Delay(200);


    NVIC_SystemReset();
    while (1) {}
    return true;
}
