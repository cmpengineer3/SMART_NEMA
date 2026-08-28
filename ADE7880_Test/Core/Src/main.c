/**
  ******************************************************************************
  * @file    main.c
  * @brief   FIRMWARE DIAGNOSTIK ADE7880 — hanya pembacaan sensor
  *
  *  Tujuan: mengisolasi pembacaan ADE7880 dari modem EC25, MQTT, FRAM, dan
  *  relay, supaya bisa dipastikan apakah ketidakstabilan pembacaan berasal
  *  dari chip/hardware atau dari interaksi dengan subsistem lain.
  *
  *  TETAP DIPERTAHANKAN (sesuai permintaan):
  *    - Kristal eksternal HSE + PLL x9  → SYSCLK 72 MHz  (identik project asli)
  *    - Watchdog IWDG Prescaler 256 / Reload 4095 (~26 detik)
  *    - Peta pin ADE7880 identik project asli
  *
  *  TIDAK ADA: USART3/modem, MQTT, relay.
  *  [STEP 2] I2C1 + FRAM AKTIF.
  *  [STEP 3] RTC + ADC + TIM AKTIF (peripheral saja, tanpa IRQ).
  *  [STEP 2] I2C1 + FRAM AKTIF — WH disimpan persisten.
  *  Jadi TIDAK ADA interrupt UART3 yang bisa menyela bit-bang SPI.
  *
  *  PETA PIN
  *    PB12  ADE7880_CS        PC6  ADE_PM1        PA9   USART1_TX (debug)
  *    PB13  SPI2_SCK          PC7  ADE_PM0        PA10  USART1_RX (debug)
  *    PB14  SPI2_MISO         PC8  ADE_RST        PA5   LED
  *    PB15  SPI2_MOSI         PC9  ADE_IRQ1       PA8   ADE_IRQ0
  *
  *  DEBUG UART : USART1, 115200 8N1
  *
  *  PERINTAH (akhiri dengan Enter, huruf besar/kecil bebas):
  *    HELP              tampilkan daftar perintah
  *    READ              baca sekali sekarang
  *    RAW               tampilkan nilai mentah terakhir
  *    CAL               tampilkan konstanta kalibrasi
  *    INT,<ms>          ubah interval baca (mis. INT,5000)
  *    POW,ON | POW,OFF  hidupkan/matikan getDataPOW (ruang 0xE5xx)
  *    PFF,ON | PFF,OFF  hidupkan/matikan getDataPFf (ruang 0xE9xx)
  *    DLY,<setup>,<hold>,<inter>   ubah timing bit-bang saat runtime
  *    RECFG             jalankan ulang ADE7880_Config()
  *    STAT              statistik: jumlah baca, jumlah nilai tidak wajar
  *    RESET             restart MCU
  ******************************************************************************
  */

#include "main.h"
#include "powermeter_ade7880.h"

/* [STEP 2 — 2026-08-28] Persistensi WH via FRAM I2C1 */
#include "i2c.h"
#include "fram.h"

/* [STEP 3 — 2026-08-28] Peripheral tambahan (RTC/ADC/TIM) — clock aktif tapi
 * belum ada interrupt yg didaftarkan. Uji apakah sekedar menyalakan mereka
 * mengganggu bit-bang SPI ADE7880. */
#include "rtc.h"
#include "adc.h"
#include "tim.h"

/* [STEP 6] Modem UART + AT sync */
#include "uart.h"
#include "modem_check.h"

/* [STEP 7 — PJUTS style] MQTT boot */
#include "mqtt.h"
#include "config.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/* ── Handle peripheral ────────────────────────────────────────────────────── */
UART_HandleTypeDef huart1;
IWDG_HandleTypeDef hiwdg;

/* [STEP 5 — 2026-08-28] USART3 (modem EC25) + UART4 (header J5) di-init supaya
 * peripheral clock + IRQn AKTIF, tapi HAL_UART_Receive_IT SENGAJA tidak
 * dipanggil → RXNE interrupt enable = 0 → ISR tidak akan fire meski ada
 * byte masuk. Modem tetap OFF (EN3V8 = LOW). Yang mau kita uji: apakah
 * sekedar menyalakan peripheral clock USART3/UART4 mengganggu bit-bang
 * SPI ke ADE7880. */
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;

#define FW_VERSION "2026.08.27"

/* ── Prototipe ────────────────────────────────────────────────────────────── */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);   /* [STEP 5] */
static void MX_UART4_Init(void);         /* [STEP 5] */
static void MX_IWDG_Init(void);

/* ── Status aplikasi ──────────────────────────────────────────────────────── */
static uint32_t read_interval_ms = 5000;
static uint32_t last_read_tick   = 0;

/* [STEP 7] Device UID + globals mirror New_KWH */
char device_uid[24] = "KWH012501705";
static char        last_topic[128]   = {0};
static char        last_payload[512] = {0};
static ModemStatus mstat;
static int         retry_count_local = 0;
static int         total_retry       = 0;
static uint32_t read_count       = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 *  [STEP 1 — 2026-08-28]  Sinkronisasi variabel & format serial dgn New_KWH
 *
 *  Tujuan langkah ini SEMURNI menyamakan struktur data + tampilan debug.
 *  Belum ada fitur baru dari sisi hardware/protokol. Kalau setelah ini
 *  bacaan tetap bersih seperti sebelumnya, artinya perubahan pure-software
 *  ini aman dan kita boleh lanjut Langkah 2 (FRAM).
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    float VRms_R, VRms_S, VRms_T;
    float IRms_R, IRms_S, IRms_T;
    float Pf_R,   Pf_S,   Pf_T;
    float Freq_R, Freq_S, Freq_T;
    float WH_R,   WH_S,   WH_T;
    int   WHC_R,  WHC_S,  WHC_T;
    float Pow_R,  Pow_S,  Pow_T;   /* Daya AKTIF (W) — dari register ADE     */
    float VA_R,   VA_S,   VA_T;    /* Daya SEMU (VA) — dihitung V × I        */
} pwr_value_t;

/* ══════════════════════════════════════════════════════════════════════════
 *  [STEP 4] Relay DO1 control — ACTIVE HIGH
 *
 *  Skematik sheet 2/4:
 *      PA1 --[R6 220R]--> gate Q1 BSS138L (N-channel low-side)
 *                                     |
 *                                     +- drain --> koil relay --> +5V
 *
 *  Gate HIGH = MOSFET ON = koil aktif = RELAY ON.
 * ══════════════════════════════════════════════════════════════════════════ */
#define RELAY_ACTIVE_HIGH   1

static uint8_t relay_state = 0;   /* 0 = OFF, 1 = ON */

//static void Relay_Set(uint8_t on)
//{
//#if RELAY_ACTIVE_HIGH
//    HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin,
//                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
//#else
//    HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin,
//                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
//#endif
//    relay_state = on ? 1 : 0;
//}

static void Relay_Set(uint8_t on)
{
    if (on)
        HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
    relay_state = on ? 1 : 0;
}

static bool has_hcmd(const char *data_obj, char c)
{
    const char *p = strstr(data_obj, "\"H\"");
    if (p == NULL) return false;
    p += 3;
    while (*p == ' ') p++;
    if (*p != ':') return false;
    p++;
    while (*p == ' ') p++;
    if (*p != '\"') return false;
    p++;
    return (*p == c);
}

static const char *Nama_Hari(int y, int m, int d)
{
    static const char *hari[] = {
        "Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"
    };
    static const int t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
    if (m < 3) y -= 1;
    if (m < 1 || m > 12) return "-";
    int idx = (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
    return hari[idx];
}


static MQTT_StatusTypeDef Publish_Full_Payload(pwr_value_t *pv, float wh)
{

    float PV_R = pv->Pow_R, PV_S = pv->Pow_S, PV_T = pv->Pow_T;   /* aktif (W)  */
    float VA_R = pv->VA_R,  VA_S = pv->VA_S,  VA_T = pv->VA_T;    /* semu (VA)  */

    /* Total daya aktif */
    float W_tot = PV_R + PV_S + PV_T;

    /* Sinyal dBm dari CSQ (99 = unknown → 0) */
    int sig_dbm = (mstat.csq == 99 || mstat.csq == 0) ? 0 : (-113 + 2 * mstat.csq);

    /* 1) Susun DATA object dulu (untuk hitung CRC-nya) */
    static char data_obj[768];
    snprintf(data_obj, sizeof(data_obj),
        "{\"H\":\"K\","
        "\"La\":%.6f,\"Lo\":%.6f,"
        "\"VR\":%.2f,\"VS\":%.2f,\"VT\":%.2f,"
        "\"IR\":%.3f,\"IS\":%.3f,\"IT\":%.3f,"
        "\"PFR\":%.2f,\"PFS\":%.2f,\"PFT\":%.2f,"
        "\"PVR\":%.2f,\"PVS\":%.2f,\"PVT\":%.2f,"
        "\"VAR\":%.2f,\"VAS\":%.2f,\"VAT\":%.2f,"
        "\"WH\":%.3f,\"W\":%.2f,"
        "\"Frek\":%.3f,\"Temp\":%.2f,\"Sig\":%d,"
        "\"DO1\":%u,\"DO2\":0,\"DI2\":0,\"SEC\":%d}",
        (double)LATITUDE, (double)LONGITUDE,
        (double)pv->VRms_R, (double)pv->VRms_S, (double)pv->VRms_T,
        (double)pv->IRms_R, (double)pv->IRms_S, (double)pv->IRms_T,
        (double)pv->Pf_R,   (double)pv->Pf_S,   (double)pv->Pf_T,
        (double)PV_R,       (double)PV_S,       (double)PV_T,
        (double)VA_R,       (double)VA_S,       (double)VA_T,
        (double)wh,         (double)W_tot,
        (double)pv->Freq_R, 0.0, sig_dbm,
        (unsigned)relay_state, 0);

    /* 2) Hitung CRC dari DATA object, format "0xXXXX" */
    uint16_t crc = Modbus_CRC16((const unsigned char *)data_obj, strlen(data_obj));
    char crc_str[8];
    snprintf(crc_str, sizeof(crc_str), "0x%04X", crc);

    /* 3) Rangkai payload penuh: CRC + DATA + LU + Ver + COM + IMSI + IMEI */
    static char full[1024];
    snprintf(full, sizeof(full),
        "{\"CRC\":\"%s\",\"DATA\":%s,"
        "\"LU\":{\"dy\":\"%s\",\"dt\":\"%04d:%02d:%02d\",\"tm\":\"%02d:%02d:%02d\",\"sc\":\"ntp\"},"
        "\"Ver\":\"%s\",\"COM\":\"GSM\",\"IMSI\":\"%s\",\"IMEI\":\"%s\"}",
        crc_str, data_obj,
        Nama_Hari(year, month, day),
        year, month, day, hour, minute, second,
        FW_VERSION, mstat.imsi, mstat.imei);

    /* 4) Publish langsung (BUKAN MQTT_PublishWithCRC, karena format beda) */
    MQTT_StatusTypeDef st = MQTT_Publish(mqtt_topic_pub, full, 1, 0);
    if (st == MQTT_OK)
        HAL_UART_Transmit(&huart1, (uint8_t*)"[PUBLISH] payload terkirim\r\n", 28, 500);
    else
        HAL_UART_Transmit(&huart1, (uint8_t*)"[PUBLISH] payload GAGAL\r\n", 25, 500);
    return st;
}


static pwr_value_t pwr_val;
static pwr_value_t last_pwr_val;   /* untuk deteksi stuck (belum dipakai)    */

/* Akumulator energi per fasa (Wh) + total */
static float Wh_R = 0.0f, Wh_S = 0.0f, Wh_T = 0.0f;
static float WH   = 0.0f;

/* Waktu baca terakhir — untuk hitung elapsed → updateWh */
static uint32_t last_ade_read_tick = 0;

/* Mirip updateWh() di KWH.c New_KWH. Basisnya VA (daya semu). */
static void updateWh(uint32_t elapsed_ms)
{
    if (elapsed_ms == 0) return;
    float hours = (float)elapsed_ms / 3600000.0f;
    if (pwr_val.VA_R > 0.0f) Wh_R += pwr_val.VA_R * hours;
    if (pwr_val.VA_S > 0.0f) Wh_S += pwr_val.VA_S * hours;
    if (pwr_val.VA_T > 0.0f) Wh_T += pwr_val.VA_T * hours;
    pwr_val.WH_R = Wh_R;
    pwr_val.WH_S = Wh_S;
    pwr_val.WH_T = Wh_T;
    WH           = Wh_R + Wh_S + Wh_T;
}

static void resetWH(void)
{
    Wh_R = Wh_S = Wh_T = 0.0f;
    WH   = 0.0f;
    pwr_val.WH_R = pwr_val.WH_S = pwr_val.WH_T = 0.0f;
}

/* Format IDENTIK dengan Debug_Print_Sensor() di New_KWH main.c */
static void Debug_Print_Sensor(pwr_value_t *pv, float wh)
{
    char b[140];
    int  n;
    /* PW = daya AKTIF (Watt, dari ADE) | VA = daya SEMU (V × I) */
    n = sprintf(b, "R: V=%3.2f I=%2.3f PF=%1.2f PW=%2.2f VA=%2.2f\r\n",
                pv->VRms_R, pv->IRms_R, pv->Pf_R, pv->Pow_R, pv->VA_R);
    HAL_UART_Transmit(&huart1, (uint8_t*)b, n, 1000);
    n = sprintf(b, "S: V=%3.2f I=%2.3f PF=%1.2f PW=%2.2f VA=%2.2f\r\n",
                pv->VRms_S, pv->IRms_S, pv->Pf_S, pv->Pow_S, pv->VA_S);
    HAL_UART_Transmit(&huart1, (uint8_t*)b, n, 1000);
    n = sprintf(b, "T: V=%3.2f I=%2.3f PF=%1.2f PW=%2.2f VA=%2.2f\r\n",
                pv->VRms_T, pv->IRms_T, pv->Pf_T, pv->Pow_T, pv->VA_T);
    HAL_UART_Transmit(&huart1, (uint8_t*)b, n, 1000);
    n = sprintf(b, "Frek=%2.2f  WH=%3.5f\r\n", pv->Freq_R, wh);
    HAL_UART_Transmit(&huart1, (uint8_t*)b, n, 1000);
}

/* Statistik nilai tidak wajar (batas hanya untuk MENANDAI, tidak menyaring —
 * di firmware diagnostik kita ingin melihat nilai apa adanya). */
static uint32_t bad_V = 0, bad_I = 0, bad_F = 0;
#define V_LO   90.0f
#define V_HI  300.0f
#define I_HI  100.0f
#define F_LO   45.0f
#define F_HI   55.0f

/* Buffer perintah dari UART1 (diisi ISR, diproses di superloop) */
/* rx_byte dipindah ke uart.c (uart1_dbg_byte) — dispatched via UART_OnDebugByte */
static char     cmd_buf[64];
static uint8_t  cmd_idx = 0;
static volatile bool cmd_ready = false;

/* ══════════════════════════════════════════════════════════════════════════
 *  Cetak ke UART debug
 * ══════════════════════════════════════════════════════════════════════════ */
static void P(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, strlen(s), 1000);
}

static void Pf(const char *fmt, ...)
{
    char b[200];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n > 0) HAL_UART_Transmit(&huart1, (uint8_t *)b, (uint16_t)n, 1000);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Konversi mentah → bernilai (sama persis dengan KWH.c project asli)
 * ══════════════════════════════════════════════════════════════════════════ */

/* ADE7880 menyimpan nilai 24-bit di dalam wadah 32-bit. Rumus sign-extend
 * yang dipakai driver: (int)(raw << 8) / 256 */
static int32_t sx24(uint32_t raw)
{
    int32_t v = (int32_t)(raw << 8);
    return v / 256;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Satu siklus baca + cetak
 * ══════════════════════════════════════════════════════════════════════════ */
static void DoRead(void)
{
    float Vr, Vs, Vt, Ir, Is, It, In;
    float pfr, pfs, pft, fr, fs, ft;
    float Pr = 0, Ps = 0, Pt = 0;

    uint32_t t0 = HAL_GetTick();

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

    ADE7880_getData(&Vr, &Vs, &Vt, &Ir, &Is, &It, &In);
    ADE7880_getDataPFf(&pfr, &pfs, &pft, &fr, &fs, &ft);
    ADE7880_getDataPOW(&Pr, &Ps, &Pt);          /* dilewati bila ade_en_getPOW=0 */

    uint32_t dt = HAL_GetTick() - t0;
    read_count++;

    /* ── Elapsed sejak baca terakhir (untuk akumulasi WH) ────────────────── */
    uint32_t now_tick = HAL_GetTick();
    uint32_t elapsed  = (last_ade_read_tick == 0) ? 0 : (now_tick - last_ade_read_tick);
    last_ade_read_tick = now_tick;

    /* ── Kalibrasi (rumus identik project asli / New_KWH) ────────────────── */
    float Vrc = Vr * GAIN_VRMSR + OFFS_VRMSR;
    float Vsc = Vs * GAIN_VRMSS + OFFS_VRMSS;
    float Vtc = Vt * GAIN_VRMST + OFFS_VRMST;
    float Irc = Ir * GAIN_IRMSR + OFFS_IRMSR - 0.16f;
    float Isc = Is * GAIN_IRMSS + OFFS_IRMSS - 0.16f;
    float Itc = It * GAIN_IRMST + OFFS_IRMST - 0.16f;

    /* [PERBAIKAN 2026-08-28] Buang tampilan "-0.000".
     *
     * Setelah OFFS_IRMSR dikalibrasi supaya no-load ≈ 0, hasil per siklus
     * berayun sedikit di kedua sisi nol (mis. +0,000163 / -0,000109). Nilai
     * negatif yang lebih kecil dari 0,0005 masih dicetak oleh printf %.3f
     * sebagai "-0.000" karena sign bit float-nya menyala.
     *
     * CT bersifat searah — nilai I negatif secara fisik tidak berarti — jadi
     * aman dipangkas ke 0. Sekaligus menghilangkan VA "gaib" (mis. 0,06 saat
     * I sebetulnya nol) karena VA di bawah dihitung dari I yang sudah dipangkas. */
    if (Irc < 0.0f) Irc = 0.0f;
    if (Isc < 0.0f) Isc = 0.0f;
    if (Itc < 0.0f) Itc = 0.0f;

    /* ── Daya semu VA = V × I per fasa (dihitung, tidak diambil dari chip) ──
     *
     *  ADE7880 punya register VA sendiri (AVA/BVA/CVA di 0xE5xx), tapi
     *  di firmware ini kita ambil pendekatan yang paling gampang dibaca:
     *  hitung sendiri dari hasil kalibrasi V dan I. Sinkron dengan cara
     *  KWH.c/KWH_NoMQTT menghitung VA, sehingga angkanya bisa langsung
     *  dibandingkan silang.
     *
     *  Kalau I hasil kalibrasi bernilai sedikit negatif (noise di sekitar
     *  nol), VA dipaksa 0 supaya tidak muncul angka aneh negatif. */
    float VAr = Vrc * Irc;  if (VAr < 0.0f) VAr = 0.0f;
    float VAs = Vsc * Isc;  if (VAs < 0.0f) VAs = 0.0f;
    float VAt = Vtc * Itc;  if (VAt < 0.0f) VAt = 0.0f;

    /* ── Isi struct pwr_val — sinkron dgn New_KWH getElectricValue() ─────── */
    pwr_val.VRms_R = Vrc;  pwr_val.VRms_S = Vsc;  pwr_val.VRms_T = Vtc;
    pwr_val.IRms_R = Irc;  pwr_val.IRms_S = Isc;  pwr_val.IRms_T = Itc;
    pwr_val.Pf_R   = pfr;  pwr_val.Pf_S   = pfs;  pwr_val.Pf_T   = pft;
    pwr_val.Freq_R = fr;   pwr_val.Freq_S = fs;   pwr_val.Freq_T = ft;
    pwr_val.Pow_R  = Pr;   pwr_val.Pow_S  = Ps;   pwr_val.Pow_T  = Pt;
    pwr_val.VA_R   = VAr;  pwr_val.VA_S   = VAs;  pwr_val.VA_T   = VAt;
    pwr_val.WHC_R  = 1;    pwr_val.WHC_S  = 1;    pwr_val.WHC_T  = 1;

    /* ── Akumulasi WH memakai elapsed nyata ──────────────────────────────── */
    updateWh(elapsed);

    /* ── [STEP 2] Simpan WH ke FRAM (persisten) ─────────────────────────── */
    WattH.m_float = WH;
    WritemByte_FRAM(addr_energy, WattH.m_bytes);

    /* ── Tandai nilai tidak wajar (hanya dihitung, tidak diubah) ── */
    if (Vrc > V_HI || Vsc > V_HI || Vtc > V_HI) bad_V++;
    if (Irc > I_HI || Isc > I_HI || Itc > I_HI) bad_I++;
    if (fr < F_LO || fr > F_HI)                 bad_F++;

    /* ── Cetak ── */
    Pf("\r\n===== #%lu  t=%lu ms  durasi=%lu ms =====\r\n",
       (unsigned long)read_count, (unsigned long)HAL_GetTick(), (unsigned long)dt);

    P("-- RAW (langsung dari SPI, sebelum konversi) --\r\n");
    Pf("  AVRMS=0x%08lX (%9ld)   AIRMS=0x%08lX (%9ld)\r\n",
       (unsigned long)ade_raw_AVRMS, (long)sx24(ade_raw_AVRMS),
       (unsigned long)ade_raw_AIRMS, (long)sx24(ade_raw_AIRMS));
    Pf("  BVRMS=0x%08lX (%9ld)   BIRMS=0x%08lX (%9ld)\r\n",
       (unsigned long)ade_raw_BVRMS, (long)sx24(ade_raw_BVRMS),
       (unsigned long)ade_raw_BIRMS, (long)sx24(ade_raw_BIRMS));
    Pf("  CVRMS=0x%08lX (%9ld)   CIRMS=0x%08lX (%9ld)\r\n",
       (unsigned long)ade_raw_CVRMS, (long)sx24(ade_raw_CVRMS),
       (unsigned long)ade_raw_CIRMS, (long)sx24(ade_raw_CIRMS));
    Pf("  NIRMS=0x%08lX (%9ld)\r\n",
       (unsigned long)ade_raw_NIRMS, (long)sx24(ade_raw_NIRMS));

    if (ade_en_getPFf)
        Pf("  APF=0x%04X BPF=0x%04X CPF=0x%04X | APER=0x%04X BPER=0x%04X CPER=0x%04X\r\n",
           ade_raw_APF, ade_raw_BPF, ade_raw_CPF,
           ade_raw_APER, ade_raw_BPER, ade_raw_CPER);
    if (ade_en_getPOW)
        Pf("  AWATT=0x%08lX BWATT=0x%08lX CWATT=0x%08lX\r\n",
           (unsigned long)ade_raw_AWATT, (unsigned long)ade_raw_BWATT,
           (unsigned long)ade_raw_CWATT);

    P("-- HASIL KALIBRASI (format sinkron New_KWH) --\r\n");
    Debug_Print_Sensor(&pwr_val, WH);
    /* Info tambahan yg tidak ada di format New_KWH — dipertahankan untuk diagnosa */
    Pf("  (N: I=%7.3f | TOTAL P=%.2f W | TOTAL VA=%.2f)\r\n",
       In * GAIN_IRMSN, Pr + Ps + Pt, VAr + VAs + VAt);

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Perintah UART
 * ══════════════════════════════════════════════════════════════════════════ */
static void ShowHelp(void)
{
    P("\r\n=========== ADE7880 TEST — DAFTAR PERINTAH ===========\r\n"
      "  HELP                    daftar perintah ini\r\n"
      "  READ                    baca sekali sekarang\r\n"
      "  RAW                     nilai mentah terakhir\r\n"
      "  CAL                     konstanta kalibrasi\r\n"
      "  STAT                    statistik pembacaan\r\n"
      "  INT,<ms>                interval baca, mis. INT,5000\r\n"
      "  POW,ON | POW,OFF        getDataPOW  (ruang 0xE5xx)\r\n"
      "  PFF,ON | PFF,OFF        getDataPFf  (ruang 0xE9xx)\r\n"
      "  DLY,<setup>,<hold>,<inter>   timing bit-bang\r\n"
      "  RECFG                   ulang ADE7880_Config()\r\n"
      "  RESET                   restart MCU\r\n"
      "  WHRST                   reset akumulasi WH (RAM+FRAM)\r\n"
      "  RELAY,0 | RELAY,1       DO1 relay OFF / ON (aktif-HIGH)\r\n"
      "======================================================\r\n");
}

static void ShowStat(void)
{
    Pf("\r\n--- STATISTIK ---\r\n"
       "  jumlah baca      : %lu\r\n"
       "  V di luar wajar  : %lu\r\n"
       "  I di luar wajar  : %lu\r\n"
       "  F di luar wajar  : %lu\r\n"
       "  interval         : %lu ms\r\n"
       "  getData(0x43xx)  : %s\r\n"
       "  getDataPFf(0xE9xx): %s\r\n"
       "  getDataPOW(0xE5xx): %s\r\n"
       "  timing setup/hold/inter : %lu / %lu / %lu\r\n",
       (unsigned long)read_count, (unsigned long)bad_V,
       (unsigned long)bad_I, (unsigned long)bad_F,
       (unsigned long)read_interval_ms,
       ade_en_getData ? "ON" : "OFF",
       ade_en_getPFf  ? "ON" : "OFF",
       ade_en_getPOW  ? "ON" : "OFF",
       (unsigned long)ade_dly_cs_setup, (unsigned long)ade_dly_cs_hold,
       (unsigned long)ade_dly_interframe);
    Pf("  Relay DO1        : %s\r\n", relay_state ? "ON" : "OFF");
}

static void ShowCal(void)
{
    Pf("\r\n--- KONSTANTA KALIBRASI ---\r\n"
       "  GAIN_VRMS R/S/T : %.12f / %.12f / %.12f\r\n"
       "  OFFS_VRMS R/S/T : %.6f / %.6f / %.6f\r\n"
       "  GAIN_IRMS R/S/T : %.12f / %.12f / %.12f\r\n"
       "  OFFS_IRMS R/S/T : %.6f / %.6f / %.6f\r\n"
       "  (arus juga dikurangi tetap 0.16 A)\r\n",
       GAIN_VRMSR, GAIN_VRMSS, GAIN_VRMST,
       OFFS_VRMSR, OFFS_VRMSS, OFFS_VRMST,
       GAIN_IRMSR, GAIN_IRMSS, GAIN_IRMST,
       OFFS_IRMSR, OFFS_IRMSS, OFFS_IRMST);
}

static void ShowRaw(void)
{
    P("\r\n--- NILAI MENTAH TERAKHIR ---\r\n");
    Pf("  AVRMS=0x%08lX  BVRMS=0x%08lX  CVRMS=0x%08lX\r\n",
       (unsigned long)ade_raw_AVRMS, (unsigned long)ade_raw_BVRMS,
       (unsigned long)ade_raw_CVRMS);
    Pf("  AIRMS=0x%08lX  BIRMS=0x%08lX  CIRMS=0x%08lX  NIRMS=0x%08lX\r\n",
       (unsigned long)ade_raw_AIRMS, (unsigned long)ade_raw_BIRMS,
       (unsigned long)ade_raw_CIRMS, (unsigned long)ade_raw_NIRMS);
    Pf("  APF=0x%04X  BPF=0x%04X  CPF=0x%04X\r\n",
       ade_raw_APF, ade_raw_BPF, ade_raw_CPF);
    Pf("  APER=0x%04X BPER=0x%04X CPER=0x%04X\r\n",
       ade_raw_APER, ade_raw_BPER, ade_raw_CPER);
    Pf("  AWATT=0x%08lX BWATT=0x%08lX CWATT=0x%08lX\r\n",
       (unsigned long)ade_raw_AWATT, (unsigned long)ade_raw_BWATT,
       (unsigned long)ade_raw_CWATT);
}

/* Bandingkan token tanpa peduli besar/kecil huruf */
static int ieq(const char *a, const char *b)
{
    while (*a && *b) {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) return 0;
        a++; b++;
    }
    return *a == *b;
}

static void ProcessCmd(void)
{
    if (!cmd_ready) return;

    char  work[64];
    strncpy(work, cmd_buf, sizeof(work) - 1);
    work[sizeof(work) - 1] = '\0';

    /* pecah berdasarkan koma */
    char *tok[5] = {0};
    int   nt = 0;
    char *p = work;
    while (*p == ' ') p++;
    tok[nt++] = p;
    while (*p && nt < 5) {
        if (*p == ',') { *p = '\0'; p++; while (*p == ' ') p++; tok[nt++] = p; }
        else p++;
    }
    /* buang spasi/CR di ujung tiap token */
    for (int i = 0; i < nt; i++) {
        char *e = tok[i] + strlen(tok[i]);
        while (e > tok[i] && (e[-1] == ' ' || e[-1] == '\r' || e[-1] == '\n')) e--;
        *e = '\0';
    }

    if      (ieq(tok[0], "HELP"))  ShowHelp();
    else if (ieq(tok[0], "READ"))  DoRead();
    else if (ieq(tok[0], "RAW"))   ShowRaw();
    else if (ieq(tok[0], "CAL"))   ShowCal();
    else if (ieq(tok[0], "STAT"))  ShowStat();
    else if (ieq(tok[0], "RECFG")) { P("\r\n[CFG] konfigurasi ulang ADE7880...\r\n");
                                     ADE7880_Config(); }
    else if (ieq(tok[0], "RESET")) { P("\r\n[SYS] restart...\r\n"); HAL_Delay(100);
                                     NVIC_SystemReset(); }
    else if (ieq(tok[0], "WHRST")) {
        resetWH();
        WattH.m_float = 0.0f;
        WritemByte_FRAM(addr_energy, WattH.m_bytes);  /* [STEP 2] timpa FRAM */
        P("\r\n[SYS] WH direset ke 0 (RAM + FRAM)\r\n");
    }
    else if (ieq(tok[0], "RELAY") && nt >= 2) {
        if (tok[1][0] == '0' && tok[1][1] == '\0') {
            Relay_Set(0); P("\r\n[REL] RELAY OFF\r\n");
        } else if (tok[1][0] == '1' && tok[1][1] == '\0') {
            Relay_Set(1); P("\r\n[REL] RELAY ON\r\n");
        } else {
            P("\r\n[ERR] RELAY,0  atau  RELAY,1\r\n");
        }
    }
    else if (ieq(tok[0], "INT") && nt >= 2) {
        long v = atol(tok[1]);
        if (v >= 200 && v <= 600000) { read_interval_ms = (uint32_t)v;
            Pf("\r\n[SET] interval = %lu ms\r\n", (unsigned long)read_interval_ms); }
        else P("\r\n[ERR] interval 200..600000 ms\r\n");
    }
    else if (ieq(tok[0], "POW") && nt >= 2) {
        ade_en_getPOW = ieq(tok[1], "ON") ? 1 : 0;
        Pf("\r\n[SET] getDataPOW (0xE5xx) = %s\r\n", ade_en_getPOW ? "ON" : "OFF");
    }
    else if (ieq(tok[0], "PFF") && nt >= 2) {
        ade_en_getPFf = ieq(tok[1], "ON") ? 1 : 0;
        Pf("\r\n[SET] getDataPFf (0xE9xx) = %s\r\n", ade_en_getPFf ? "ON" : "OFF");
    }
    else if (ieq(tok[0], "DLY") && nt >= 4) {
        ade_dly_cs_setup   = (uint32_t)atol(tok[1]);
        ade_dly_cs_hold    = (uint32_t)atol(tok[2]);
        ade_dly_interframe = (uint32_t)atol(tok[3]);
        Pf("\r\n[SET] timing setup/hold/inter = %lu / %lu / %lu\r\n",
           (unsigned long)ade_dly_cs_setup, (unsigned long)ade_dly_cs_hold,
           (unsigned long)ade_dly_interframe);
    }
    else P("\r\n[ERR] perintah tidak dikenali — ketik HELP\r\n");

    cmd_idx = 0;
    cmd_ready = false;
}

/* ISR RX UART1 — sengaja dibuat SANGAT RINGAN (tanpa strstr / parsing) */
/* [STEP 6] HAL_UART_RxCpltCallback dipindah ke uart.c (yang meng-handle
 *          huart3 modem juga). Kita hanya override HOOK aplikasi berikut:
 *   - UART_OnDebugByte  → jalur byte USART1 debug (ganti ISR lama)
 *   - UART_DebugPoll    → dipanggil dari wait-loop AT command supaya
 *                         perintah serial tetap responsif
 *   - UART_WatchdogRefresh → kick IWDG dari wait-loop AT command       */

void UART_OnDebugByte(uint8_t b)
{
    if (cmd_ready) return;
    if (b == '\r' || b == '\n') {
        if (cmd_idx > 0) { cmd_buf[cmd_idx] = '\0'; cmd_ready = true; }
    } else if (cmd_idx < sizeof(cmd_buf) - 1) {
        cmd_buf[cmd_idx++] = (char)b;
    } else {
        cmd_idx = 0;
    }
}

void UART_DebugPoll(void)      { ProcessCmd(); }
void UART_WatchdogRefresh(void){ HAL_IWDG_Refresh(&hiwdg); }

/* ══════════════════════════════════════════════════════════════════════════
 *  main
 * ══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();     /* HSE (kristal eksternal) + PLL x9 = 72 MHz */

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();    /* [STEP 5] PB10 TX / PB11 RX @115200, IRQn ON, RX BELUM diarm */
    MX_UART4_Init();          /* [STEP 5] PC10 TX / PC11 RX @115200, IRQn ON, RX BELUM diarm */
    MX_I2C1_Init();           /* [STEP 2] I2C1 PB6/PB7 → FRAM 0xA0 */
    MX_RTC_Init();            /* [STEP 3] RTC (sumber clock LSI, sudah aktif) */
    MX_ADC1_Init();            /* [STEP 3] ADC1 kanal internal temp sensor    */
    MX_TIM2_Init();            /* [STEP 3] TIM2 base 1 kHz — belum di-Start   */
    MX_IWDG_Init();           /* ~26 detik */

    UART_StartDebugRx();

    P("\r\n\r\n");
    P("##########################################################\r\n");
    P("#   ADE7880 READ TEST  —  firmware diagnostik            #\r\n");
    P("#   STM32F103RGTx @72MHz (HSE+PLLx9)  IWDG ~26 s         #\r\n");
    P("#   [STEP 2] I2C1 + FRAM aktif  [STEP 3] RTC/ADC/TIM    #\r\n");
    P("#   [STEP 4] Relay DO1 aktif                            #\r\n");
    P("#   [STEP 5] USART3+UART4 clocked, IRQn ON              #\r\n");
    P("#   [STEP 6] Modem EC25 ON + AT sync 115200            #\r\n");
    P("#   [STEP 7] MQTT (PJUTS style, URC 90s) ke broker      #\r\n");
    P("#   [STEP 3] RTC + ADC + TIM aktif (idle peripheral)   #\r\n");
    P("##########################################################\r\n");
    Pf("  getDataPOW (0xE5xx) default : %s\r\n", ade_en_getPOW ? "ON" : "OFF");
    P("  Ketik HELP lalu Enter untuk daftar perintah.\r\n\r\n");

    HAL_IWDG_Refresh(&hiwdg);
    P("[CFG] inisialisasi ADE7880...\r\n");
    ADE7880_Config();
    HAL_IWDG_Refresh(&hiwdg);

    /* ── [STEP 2] Restore WH dari FRAM (persisten antar-reset) ─────────── */
    WH = FRAM_Read_WH();
    if (WH < 0.0f || WH > 1.0e9f) WH = 0.0f;  /* guard nilai sampah pertama boot */
    Wh_R = WH / 3.0f;
    Wh_S = WH / 3.0f;
    Wh_T = WH / 3.0f;
    pwr_val.WH_R = Wh_R;
    pwr_val.WH_S = Wh_S;
    pwr_val.WH_T = Wh_T;
    Pf("[WH]  restore dari FRAM = %.5f Wh\r\n", (double)WH);

    /* [STEP 4] Relay awal OFF (aman) */
    Relay_Set(0);
    P("[REL] relay awal = OFF\r\n");

    /* ══════════════════════════════════════════════════════════════════════
     *  [STEP 6] Aktifkan RX huart3 + power-on modem EC25 + AT sync
     *
     *  Sekuens sama persis dengan New_KWH main.c:
     *    - Arm huart3 RX-IT (byte modem masuk buffer via uart.c ISR ringan)
     *    - Nyalakan 3.8V modem, LTE_PWR, LTE_RST
     *    - Tunggu ~15 detik (chunked HAL_Delay 1s + WDG kick) supaya modem
     *      sempat boot & mulai mengirim URC (RDY, +CFUN, dst.)
     *    - Modem_AutoBaudrate(115200) — sync AT command, kalau perlu
     *      ubah baud modem ke 115200
     *
     *  Yang mau kita uji: setelah modem hidup + kirim URC + ISR huart3 fire,
     *  apakah bacaan ADE tetap bersih?
     * ══════════════════════════════════════════════════════════════════════ */
    P("\r\n[STEP 6] mengaktifkan modem EC25...\r\n");
    UART_Init_Buffer(&huart3);      /* arm huart3 RX-IT */

    HAL_GPIO_WritePin(LTE_RST_GPIO_Port, LTE_RST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN3V8_GPIO_Port,   EN3V8_Pin,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(LTE_PWR_GPIO_Port, LTE_PWR_Pin, GPIO_PIN_SET);
    P("[MODEM] 3V8+PWR+RST=HIGH, tunggu boot 15 detik...\r\n");
    for (int i = 0; i < 15; i++)
    {
        HAL_Delay(1000);
        HAL_IWDG_Refresh(&hiwdg);
    }

    Modem_AutoBaudrate(115200);
    HAL_IWDG_Refresh(&hiwdg);
    P("[STEP 6] modem hidup — lanjut ke sekuens MQTT\r\n");

    /* ══════════════════════════════════════════════════════════════════════
     *  [STEP 7 — PJUTS style]
     *
     *  Sekuens MQTT PERSIS mengikuti SMART_PJUTS_CKP (yang terbukti jalan
     *  dgn modem EC25 + SIM 1nce.net):
     *
     *  1) Modem_SyncAndCheck — cek SIM/CREG/CSQ/COPS
     *  2) ATE0 × 3           — matikan echo modem
     *  3) MQTT_Init(cfg)     — daftarkan kredensial
     *  4) MQTT_START:
     *     - QMTCFG session,0,1 + keepalive,0,3600
     *     - AT+QMTDISC=0 (raw) + HAL_Delay(500)
     *     - AT+QMTCLOSE=0 (raw) + HAL_Delay(2000)
     *     - while (MQTT_Open() != OK):
     *         MQTT_CheckNetwork()
     *         AT+QMTCLOSE=0 (raw) + HAL_Delay(3000)
     *         5× → goto MQTT_START;  10× → power-cycle + reset
     *     - while (MQTT_Connect() != OK):
     *         AT+QMTDISC=0 (raw) + HAL_Delay(500)
     *         3× → goto MQTT_START;  >10 → reset
     *     - while (MQTT_Subscribe() != OK): retry
     *
     *  Perbedaan KRITIS dari New_KWH:
     *     - MQTT_Open: URC timeout 90 DETIK (bukan 5) — 1nce IoT SIM butuh
     *       ~10-30 detik dial TCP karena PDP context lambat aktif.
     *     - MQTT_Connect: URC timeout 25 detik.
     *     - Retry loop pakai AT+QMTCLOSE=0 RAW dgn HAL_Delay besar
     *       (bukan MQTT_Close() function yang fast-fail).
     */
    P("[MODEM] Sync & network check...\r\n");
    Modem_SyncAndCheck(&mstat);
    HAL_IWDG_Refresh(&hiwdg);

    for (int i = 0; i < 3; i++)
    {
        UART_SendATCommand("ATE0");
        if (UART_WaitForOK(2000)) break;
        HAL_Delay(500);
    }
    HAL_IWDG_Refresh(&hiwdg);

    /* Isi topic dari device_uid */
    sprintf(mqtt_topic_pub, "SIKLON/SMARTPJU/JKT/%s/UPLINK/CCO", device_uid);
    sprintf(mqtt_topic_sub, "SIKLON/SMARTPJU/JKT/%s/DOWNLINK",   device_uid);

    MQTT_Config mqtt_cfg = Config_GetMQTT();
    MQTT_Init(&mqtt_cfg);
    P("[MQTT] init OK\r\n");
    HAL_IWDG_Refresh(&hiwdg);

MQTT_START:
    retry_count_local = 0;
    total_retry       = 0;

    UART_SendATCommand("AT+QMTCFG=\"session\",0,1");
    UART_WaitForOK(3000);
    UART_SendATCommand("AT+QMTCFG=\"keepalive\",0,3600");
    UART_WaitForOK(3000);
    HAL_IWDG_Refresh(&hiwdg);

    /* Bersihkan session/context lama SEBELUM QMTOPEN pertama (PJUTS style: RAW) */
    UART_SendATCommand("AT+QMTDISC=0");
    HAL_Delay(500);
    HAL_IWDG_Refresh(&hiwdg);
    UART_SendATCommand("AT+QMTCLOSE=0");
    HAL_Delay(2000);
    HAL_IWDG_Refresh(&hiwdg);

    P("[MQTT] Membuka koneksi ke broker (URC timeout 90 detik)...\r\n");
    while (MQTT_Open() != MQTT_OK)
    {
        P("[MQTT] QMTOPEN gagal, retry...\r\n");
        MQTT_CheckNetwork();
        /* PJUTS: RAW AT+QMTCLOSE=0 + HAL_Delay(3000) — bukan MQTT_Close() */
        UART_SendATCommand("AT+QMTCLOSE=0");
        HAL_Delay(3000);
        HAL_IWDG_Refresh(&hiwdg);
        retry_count_local++;
        total_retry++;

        if (total_retry >= 10)
        {
            P("[MQTT] Gagal 10x total — power-cycle modem & reset MCU\r\n");
            HAL_GPIO_WritePin(EN3V8_GPIO_Port, EN3V8_Pin, GPIO_PIN_RESET);
            HAL_Delay(3000);
            NVIC_SystemReset();
            while (1) {}
        }
        if (retry_count_local >= 5)
        {
            retry_count_local = 0;
            goto MQTT_START;
        }
    }
    P("[MQTT] QMTOPEN OK\r\n");
    HAL_IWDG_Refresh(&hiwdg);

    P("[MQTT] QMTCONN (URC timeout 25 detik)...\r\n");
    while (MQTT_Connect() != MQTT_OK)
    {
        P("[MQTT] QMTCONN gagal, retry...\r\n");
        UART_SendATCommand("AT+QMTDISC=0");
        retry_count_local++;
        HAL_Delay(500);
        HAL_IWDG_Refresh(&hiwdg);
        if (retry_count_local >= 3)
        {
            retry_count_local = 0;
            goto MQTT_START;
        }
        total_retry++;
        if (total_retry > 10)
        {
            P("[MQTT] Gagal 10x total — reset MCU\r\n");
            NVIC_SystemReset();
            while (1) {}
        }
    }
    P("[MQTT] QMTCONN OK\r\n");
    HAL_IWDG_Refresh(&hiwdg);

    retry_count_local = 0;
    while (MQTT_Subscribe(mqtt_topic_sub, 1) != MQTT_OK)
    {
        P("[MQTT] Subscribe gagal, retry...\r\n");
        HAL_Delay(500);
        HAL_IWDG_Refresh(&hiwdg);
        retry_count_local++;
        if (retry_count_local >= 5)
        {
            retry_count_local = 0;
            goto MQTT_START;
        }
    }
    P("[MQTT] Subscribe DOWNLINK OK\r\n");
    HAL_IWDG_Refresh(&hiwdg);

    char reg_paylaod [128];
    snprintf(reg_paylaod, sizeof(reg_paylaod),"{\"H\":\"K\",\"CITY\":\"JKT\",\"CCO\":\"%s\",\"STATUS\":\"REG\"}",device_uid);

    if (MQTT_PublishWithCRC(mqtt_regis_pub,reg_paylaod, 1, 0) == MQTT_OK)
        P("[MQTT] Registrasi terkirim\r\n");
    else
        P("[MQTT] Registrasi GAGAL\r\n");
    HAL_Delay(50);

    P("[STEP 7] siap. Pemantauan sensor + URC modem berjalan.\r\n\r\n");

    last_read_tick = HAL_GetTick();

    while (1)
    {
        HAL_IWDG_Refresh(&hiwdg);
        UART_ProcessURC();          /* [STEP 6] scan URC modem di superloop */
        ProcessCmd();

        if ((HAL_GetTick() - last_read_tick) >= read_interval_ms) {
            last_read_tick = HAL_GetTick();
            DoRead();
        }

		if (mqtt_data_ready)
		{
		  if (MQTT_ProcessIncoming(last_topic,   sizeof(last_topic),
								   last_payload, sizeof(last_payload)))
		  {
			  /* Tampilkan downlink yang diterima */
			  P("[DOWNLINK] diterima: ");
			  P(last_payload);
			  P("\r\n");

			  char data_obj_in[512];
			  if (MQTT_ExtractDataObject(last_payload, data_obj_in, sizeof(data_obj_in)))
			  {
				  if (has_hcmd(data_obj_in, 'R'))
				  {
					  P("[DOWNLINK] H:R -> kirim data sekarang\r\n");
					  Publish_Full_Payload(&pwr_val, WH);
				  }
				  else if (has_hcmd(data_obj_in, 'S'))
				  {
					  char *pdo = strstr(data_obj_in, "\"DO1\"");
					  if (pdo != NULL)
					  {
						  pdo = strchr(pdo, ':');
						  if (pdo != NULL)
						  {
							  pdo++;
							  while (*pdo == ' ') pdo++;
							  int do1 = atoi(pdo);
							  Relay_Set(do1 ? 1 : 0);

							  char m[48];
							  int mn = sprintf(m, "[DOWNLINK] H:S -> relay %s\r\n",
											   relay_state ? "ON" : "OFF");
							  HAL_UART_Transmit(&huart1, (uint8_t*)m, mn, 500);

							  char ack[64];
							  snprintf(ack, sizeof(ack),
									   "{\"UID\":\"%s\",\"DO1\":%u}", device_uid, (unsigned)relay_state);
							  MQTT_PublishWithCRC(mqtt_topic_pub, ack, 1, 0);
						  }
					  }
				  }
				  else
				  {
					  P("[DOWNLINK] perintah tidak dikenali\r\n");
				  }
			  }

			  mqtt_data_ready = false;
		  }
		}
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Clock — IDENTIK dengan project asli (kristal eksternal HSE + PLL x9)
 * ══════════════════════════════════════════════════════════════════════════ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType   = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState         = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue   = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState         = RCC_HSI_ON;
    RCC_OscInitStruct.LSIState         = RCC_LSI_ON;       /* sumber clock IWDG */
    RCC_OscInitStruct.PLL.PLLState     = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource    = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL       = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

/* ══════════════════════════════════════════════════════════════════════════
 *  GPIO — peta pin IDENTIK project asli (hanya yang dipakai ADE + LED)
 * ══════════════════════════════════════════════════════════════════════════ */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* ── [STEP 4] Set semua output ke LOW dulu sebelum dikonfigurasi ────
     *
     * Urutan: WritePin dulu (mengatur latch), lalu HAL_GPIO_Init (mengatur
     * mode). Dengan begitu tidak ada glitch HIGH sesaat saat pin berpindah
     * dari input default ke output. Modem EN3V8 & LTE_PWR & LTE_RST DIPAKSA
     * LOW → modem TETAP MATI di Step 4. */
    HAL_GPIO_WritePin(GPIOA,
        RELAY_Pin | LED_Pin | EN10_Pin | EN3V8_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,
        LTE_PWR_Pin | LTE_RST_Pin | MEM_WP_Pin,     GPIO_PIN_RESET);

    /* Output push-pull di GPIOA: RELAY, LED, EN10, EN3V8 */
    g.Pin   = RELAY_Pin | LED_Pin | EN10_Pin | EN3V8_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    /* Output open-drain di GPIOB: LTE_PWR, LTE_RST (pengaman terhadap
     * tegangan silang di dunia luar; identik gpio.c New_KWH). */
    g.Pin   = LTE_PWR_Pin | LTE_RST_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    /* Output push-pull di GPIOB: MEM_WP (write enable FRAM = LOW). */
    g.Pin   = MEM_WP_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOB, &g);

    /* Input diskret PC1/PC2 (belum dipakai — hanya set mode input). */
    g.Pin   = IN1_Pin | IN2_Pin;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &g);

    /* Pin ADE7880 (PB12/13/14/15, PC6/7/8, PA8, PC9) dikonfigurasi sendiri
     * oleh ADE7880_GPIO_Config() di dalam ADE7880_Config(). */
}

/* ══════════════════════════════════════════════════════════════════════════
 *  USART1 debug 115200 8N1  (PA9 TX / PA10 RX)
 * ══════════════════════════════════════════════════════════════════════════ */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

/* ══════════════════════════════════════════════════════════════════════════
 *  [STEP 5] USART3 modem (PB10 TX / PB11 RX) — 115200 8N1
 *
 *  DINYALAKAN peripheral clock-nya, IRQn di-enable oleh MspInit. Tapi TIDAK
 *  memanggil HAL_UART_Receive_IT() → RXNE interrupt enable tetap 0 → tidak
 *  akan ada USART3 ISR yang fire di lingkungan Step 5.
 * ══════════════════════════════════════════════════════════════════════════ */
static void MX_USART3_UART_Init(void)
{
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}

/* [STEP 5] UART4 header J5 (PC10 TX / PC11 RX) — 115200 8N1 */
static void MX_UART4_Init(void)
{
    huart4.Instance          = UART4;
    huart4.Init.BaudRate     = 115200;
    huart4.Init.WordLength   = UART_WORDLENGTH_8B;
    huart4.Init.StopBits     = UART_STOPBITS_1;
    huart4.Init.Parity       = UART_PARITY_NONE;
    huart4.Init.Mode         = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK) Error_Handler();
}

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = {0};
    if (h->Instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        g.Pin   = GPIO_PIN_9;                 /* TX */
        g.Mode  = GPIO_MODE_AF_PP;
        g.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &g);

        g.Pin  = GPIO_PIN_10;                 /* RX */
        g.Mode = GPIO_MODE_INPUT;
        g.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &g);

        HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    /* ── [STEP 5] USART3 (modem) — PB10 TX / PB11 RX ─────────────────── */
    else if (h->Instance == USART3) {
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        g.Pin   = GPIO_PIN_10;                /* TX */
        g.Mode  = GPIO_MODE_AF_PP;
        g.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &g);

        g.Pin  = GPIO_PIN_11;                 /* RX */
        g.Mode = GPIO_MODE_INPUT;
        g.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &g);

        HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
    /* ── [STEP 5] UART4 (header J5) — PC10 TX / PC11 RX ──────────────── */
    else if (h->Instance == UART4) {
        __HAL_RCC_UART4_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        g.Pin   = GPIO_PIN_10;                /* TX */
        g.Mode  = GPIO_MODE_AF_PP;
        g.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOC, &g);

        g.Pin  = GPIO_PIN_11;                 /* RX */
        g.Mode = GPIO_MODE_INPUT;
        g.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &g);

        HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  IWDG — Prescaler 256 / Reload 4095 ≈ 26 detik (identik project asli)
 * ══════════════════════════════════════════════════════════════════════════ */
static void MX_IWDG_Init(void)
{
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Reload    = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) Error_Handler();
}

/* ══════════════════════════════════════════════════════════════════════════ */
void HAL_MspInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();   /* bebaskan pin JTAG, sisakan SWD */
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
