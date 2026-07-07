/*
 * ade7880_reader.c
 *
 *  Created on: Mar 2, 2026
 *      Author: firza
 */


/**
 ******************************************************************************
 * @file    ade7880_reader.c
 * @brief   Implementasi driver pembacaan ADE7880
 ******************************************************************************
 */

/**
 ******************************************************************************
 * @file    ade7880_reader.c
 * @brief   Implementasi driver pembacaan ADE7880
 *          Direvisi berdasarkan datasheet ADE7880 Rev.A
 *
 * PERUBAHAN DARI DATASHEET:
 * [1] SPI Mode: SCK idle=LOW, data di-set sebelum rising edge
 *     Ref: "Data shifts into ADE7880 on falling edge, sampled on rising edge"
 *
 * [2] SPI Mode Selection: 3x write ke 0xEBFF (bukan sekadar toggle CS manual)
 *     Ref: "execute three SPI write operations to location 0xEBFF"
 *
 * [3] Silicon Anomaly er002: 3 write berturut-turut untuk akurasi optimal
 *     0xAD→0xE7FE, 0x3BD→0xE90C, 0x00→0xE7EF tanpa operasi lain di antara
 *
 * [4] RMS registers: 24-bit UNSIGNED (ZP format), 8 MSB selalu 0
 *     Ref: Figure 64 "AIRMS transmitted as 32-bit with 8 MSBs padded 0s"
 *
 * [5] Settling time: minimal 580ms setelah RUN=1
 *     Ref: Table 12 "Settling Time for I RMS Measurement: 580ms"
 *
 * [6] DREADY flag (STATUS0 bit 17): sinkronisasi baca dengan siklus DSP 8kHz
 *     Mencegah baca nilai 0 saat DSP sedang update register
 *
 * [7] PSM0 mode: PM0=LOW, PM1=LOW (proyek lama salah: PM0=HIGH = PSM1)
 *     Ref: Table 8 Power Mode Pin description
 *
 * [8] Formula frekuensi: f = CLKIN/(4 x PERIOD) = 4,096,000/PERIOD
 ******************************************************************************
 */

#include "ade7880_reader.h"
#include <math.h>

/* Global struct hasil pengukuran */
ADE7880_Data_t ade_data = {0};

/* Akumulasi energi software — sama seperti KWH_EC25_V1c (Wh += P/3600 per detik)
 * Menggunakan waktu konversi aktual (dt) untuk presisi lebih baik.           */
static float Wh_R = 0, Wh_S = 0, Wh_T = 0;

/* ============================================================
 * DELAY — identik dengan referensi asli
 * ============================================================ */
static void ADE7880_Delay(volatile uint32_t nCount)
{
    while(nCount > 0) { nCount--; }
}

/* ============================================================
 * GPIO INIT
 * ============================================================ */
static void ADE7880_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Output: CS, SCK, MOSI, RESET, PM0, PM1 */
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;

    GPIO_InitStructure.Pin = ADE7880_CS_PIN;
    HAL_GPIO_Init(ADE7880_CS_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = ADE7880_SCK_PIN;
    HAL_GPIO_Init(ADE7880_SCK_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = ADE7880_MOSI_PIN;
    HAL_GPIO_Init(ADE7880_MOSI_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = ADE7880_RESET_PIN;
    HAL_GPIO_Init(ADE7880_RESET_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = ADE7880_PM0_PIN;
    HAL_GPIO_Init(ADE7880_PM0_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = ADE7880_PM1_PIN;
    HAL_GPIO_Init(ADE7880_PM1_PORT, &GPIO_InitStructure);

    /* Input: MISO */
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin  = ADE7880_MISO_PIN;
    HAL_GPIO_Init(ADE7880_MISO_PORT, &GPIO_InitStructure);
}

/* ============================================================
 * SPI INIT — identik dengan referensi (ADE7880_SPIInit)
 * ============================================================ */
static void ADE7880_SPIInit(void)
{
    ADE7880_CS_SET(ADE7880_CS_PIN);
    ADE7880_RESET_HIGH;
    ADE7880_Mode0_Set;              /* PM0=HIGH, PM1=LOW */
    ADE7880_Delay(10);
    ADE7880_RESET_LOW;
    ADE7880_Delay(20000);           /* reset pulse — identik referensi */
    ADE7880_RESET_HIGH;
    ADE7880_Delay(10);

    /* Toggle CS 3x untuk masuk SPI mode — identik referensi */
    ADE7880_CS_RESET(ADE7880_CS_PIN); ADE7880_Delay(100);
    ADE7880_CS_SET(ADE7880_CS_PIN);   ADE7880_Delay(100);
    ADE7880_CS_RESET(ADE7880_CS_PIN); ADE7880_Delay(100);
    ADE7880_CS_SET(ADE7880_CS_PIN);   ADE7880_Delay(100);
    ADE7880_CS_RESET(ADE7880_CS_PIN); ADE7880_Delay(100);
    ADE7880_CS_SET(ADE7880_CS_PIN);   ADE7880_Delay(100);

    ADE7880_Delay(20000);
}

/* ============================================================
 * SPI WRITE 8-bit — identik dengan referensi
 * ============================================================ */
void ADE7880_Write8(uint8_t out)
{
    uint8_t i;
    ADE7880_Delay(10);
    for (i = 0; i < 8; i++) {
        if ((out >> (7-i)) & 0x01) ADE7880_MOSI_SET;
        else                        ADE7880_MOSI_RESET;
        ADE7880_Delay(10);
        ADE7880_SCK_RESET;
        ADE7880_Delay(10);
        ADE7880_SCK_SET;
    }
}

/* ============================================================
 * SPI WRITE 16-bit — identik dengan referensi
 * ============================================================ */
void ADE7880_Write16(uint16_t out)
{
    uint8_t i;
    for (i = 0; i < 16; i++) {
        if ((out >> (15-i)) & 0x01) ADE7880_MOSI_SET;
        else                         ADE7880_MOSI_RESET;
        ADE7880_Delay(10);
        ADE7880_SCK_RESET;
        ADE7880_Delay(10);
        ADE7880_SCK_SET;
    }
}

/* ============================================================
 * SPI READ 16-bit — identik dengan referensi
 * ============================================================ */
uint16_t ADE7880_Read16(void)
{
    uint8_t  i;
    uint16_t temp = 0;
    ADE7880_MOSI_RESET;
    ADE7880_SCK_SET;
    for (i = 0; i < 16; i++) {
        ADE7880_Delay(20);
        ADE7880_SCK_RESET;
        ADE7880_Delay(20);
        if (ADE7880_MISO == GPIO_PIN_SET) temp |= (1 << (15-i));
        ADE7880_Delay(20);
        ADE7880_SCK_SET;
    }
    return temp;
}

/* ============================================================
 * SPI READ 32-bit — identik dengan referensi
 * ============================================================ */
uint32_t ADE7880_Read32(void)
{
    uint8_t  i;
    uint32_t temp = 0;
    ADE7880_MOSI_RESET;
    ADE7880_SCK_SET;
    for (i = 0; i < 32; i++) {
        ADE7880_Delay(20);
        ADE7880_SCK_RESET;
        ADE7880_Delay(20);
        if (ADE7880_MISO == GPIO_PIN_SET) temp |= (1 << (31-i));
        ADE7880_Delay(20);
        ADE7880_SCK_SET;
    }
    return temp;
}

/* ============================================================
 * ADE7880_Config — identik dengan referensi asli
 * Panggil SEKALI saat startup
 * ============================================================ */
void ADE7880_Config(void)
{
    uint16_t ADE7880_Read_Result;

    ADE7880_GPIO_Config();

    /* Set kondisi awal — identik referensi */
    ADE7880_SCK_SET;
    ADE7880_MOSI_SET;
    ADE7880_Mode0_Set;
    ADE7880_CS_SET(ADE7880_CS_PIN);
    ADE7880_RESET_HIGH;

    ADE7880_SPIInit();

    /* Cek status chip */
    ADE7880_CS_RESET(ADE7880_CS_PIN);
    ADE7880_Delay(200);
    ADE7880_Write8(0x01);
    ADE7880_Write16(CONFIG);
    ADE7880_Read_Result = ADE7880_Read16();
    ADE7880_CS_SET(ADE7880_CS_PIN);

    ade_data.chip_status = (ADE7880_Read_Result == 0x0002) ? 1 : 0;

    /* Disable RAM Protection */
    ADE7880_CS_RESET(ADE7880_CS_PIN);
    ADE7880_Delay(200);
    ADE7880_Write8(0x00);
    ADE7880_Write16(0xE7FE);
    ADE7880_Write8(0xAD);
    ADE7880_Write8(0x00);
    ADE7880_Write16(0xE7E3);
    ADE7880_Write8(0x00);
    ADE7880_CS_SET(ADE7880_CS_PIN);

    /* Enable RAM Protection */
    ADE7880_CS_RESET(ADE7880_CS_PIN);
    ADE7880_Delay(200);
    ADE7880_Write8(0x00);
    ADE7880_Write16(0xE7FE);
    ADE7880_Write8(0xAD);
    ADE7880_Write8(0x00);
    ADE7880_Write16(0xE7E3);
    ADE7880_Write8(0x80);
    ADE7880_CS_SET(ADE7880_CS_PIN);

    /* Start DSP */
    ADE7880_CS_RESET(ADE7880_CS_PIN);
    ADE7880_Delay(200);
    ADE7880_Write8(0x00);
    ADE7880_Write16(RUN);
    ADE7880_Write16(0x0001);
    ADE7880_CS_SET(ADE7880_CS_PIN);

    /* Verifikasi RUN */
    ADE7880_CS_RESET(ADE7880_CS_PIN);
    ADE7880_Delay(200);
    ADE7880_Write8(0x01);
    ADE7880_Write16(RUN);
    ADE7880_Read_Result = ADE7880_Read16();
    ADE7880_CS_SET(ADE7880_CS_PIN);

    if (ADE7880_Read_Result != 0x0001) {
        /* DSP gagal start — ulangi dari awal (identik referensi) */
        ADE7880_Config();
    }
    /* Jika berhasil: chip_status sudah di-set di atas */
}

/* ============================================================
 * ADE7880_ReadAll — mengambil logika dari getElectricValue() di KWH.c
 * Averaging 10 sample + HAL_Delay(50) antar sample — identik referensi
 * Hasil disimpan ke struct global ade_data
 * ============================================================ */
void ADE7880_ReadAll(void)
{
    uint32_t ADE7880_Read_Result;
    int      raw32;
//    int16_t  raw16;

    float Vrms_R, Vrms_S, Vrms_T;
    float Irms_R, Irms_S, Irms_T, Irms_N;
    float Vrms_R_temp=0, Vrms_S_temp=0, Vrms_T_temp=0;
    float Irms_R_temp=0, Irms_S_temp=0, Irms_T_temp=0, Irms_N_temp=0;

    uint32_t t_start = HAL_GetTick();

    /* ---- STEP 1: Averaging 10 sample — identik KWH.c ---- */
    for (int p = 0; p < ADE_AVG_SAMPLES; p++)
    {
        /* AVRMS */
        ADE7880_CS_RESET(ADE7880_CS_PIN);
        ADE7880_Write8(0x01); ADE7880_Write16(AVRMS);
        ADE7880_Read_Result = ADE7880_Read32();
        ADE7880_CS_SET(ADE7880_CS_PIN);
        raw32 = (int)(ADE7880_Read_Result << 8); raw32 /= 256;
        Vrms_T_temp += (float)raw32;
        HAL_Delay(10);

        /* BVRMS */
        ADE7880_CS_RESET(ADE7880_CS_PIN);
        ADE7880_Write8(0x01); ADE7880_Write16(BVRMS);
        ADE7880_Read_Result = ADE7880_Read32();
        ADE7880_CS_SET(ADE7880_CS_PIN);
        raw32 = (int)(ADE7880_Read_Result << 8); raw32 /= 256;
        Vrms_S_temp += (float)raw32;
        HAL_Delay(10);

        /* CVRMS */
        ADE7880_CS_RESET(ADE7880_CS_PIN);
        ADE7880_Write8(0x01); ADE7880_Write16(CVRMS);
        ADE7880_Read_Result = ADE7880_Read32();
        ADE7880_CS_SET(ADE7880_CS_PIN);
        raw32 = (int)(ADE7880_Read_Result << 8); raw32 /= 256;
        Vrms_R_temp += (float)raw32;
        HAL_Delay(10);

        /* AIRMS → mapped ke fasa T (CT fisik di T, kabel ke pin channel A ADE7880) */
        ADE7880_CS_RESET(ADE7880_CS_PIN);
        ADE7880_Write8(0x01); ADE7880_Write16(AIRMS);
        ADE7880_Read_Result = ADE7880_Read32();
        ADE7880_CS_SET(ADE7880_CS_PIN);
        raw32 = (int)(ADE7880_Read_Result << 8); raw32 /= 256;
        Irms_T_temp += (float)raw32;
        HAL_Delay(10);

        /* BIRMS → fasa S */
        ADE7880_CS_RESET(ADE7880_CS_PIN);
        ADE7880_Write8(0x01); ADE7880_Write16(BIRMS);
        ADE7880_Read_Result = ADE7880_Read32();
        ADE7880_CS_SET(ADE7880_CS_PIN);
        raw32 = (int)(ADE7880_Read_Result << 8); raw32 /= 256;
        Irms_S_temp += (float)raw32;
        HAL_Delay(10);

        /* CIRMS → fasa R (CT terpasang di channel C) */
        ADE7880_CS_RESET(ADE7880_CS_PIN);
        ADE7880_Write8(0x01); ADE7880_Write16(CIRMS);
        ADE7880_Read_Result = ADE7880_Read32();
        ADE7880_CS_SET(ADE7880_CS_PIN);
        raw32 = (int)(ADE7880_Read_Result << 8); raw32 /= 256;
        Irms_R_temp += (float)raw32;
        HAL_Delay(10);

        /* NIRMS */
        ADE7880_CS_RESET(ADE7880_CS_PIN);
        ADE7880_Write8(0x01); ADE7880_Write16(NIRMS);
        ADE7880_Read_Result = ADE7880_Read32();
        ADE7880_CS_SET(ADE7880_CS_PIN);
        raw32 = (int)(ADE7880_Read_Result << 8); raw32 /= 256;
        Irms_N_temp += (float)raw32;
        HAL_Delay(10);

        /* Delay antar sample — identik referensi (50ms) */
        /* delay antar sample sudah termasuk dari 7x HAL_Delay(10) = 70ms, tidak perlu tambahan */
    }

    /* ---- STEP 2: Rata-rata — identik KWH.c ---- */
    Vrms_R = Vrms_R_temp / ADE_AVG_SAMPLES;
    Vrms_S = Vrms_S_temp / ADE_AVG_SAMPLES;
    Vrms_T = Vrms_T_temp / ADE_AVG_SAMPLES;
    Irms_R = Irms_R_temp / ADE_AVG_SAMPLES;
    Irms_S = Irms_S_temp / ADE_AVG_SAMPLES;
    Irms_T = Irms_T_temp / ADE_AVG_SAMPLES;
    Irms_N = Irms_N_temp / ADE_AVG_SAMPLES;

    /* ---- STEP 3: Kalibrasi tegangan — identik KWH.c ---- */
    float Vcal_R = (Vrms_R * ADE_GAIN_VRMS) + ADE_OFFS_VRMS;
    float Vcal_S = (Vrms_S * ADE_GAIN_VRMS) + ADE_OFFS_VRMS;
    float Vcal_T = (Vrms_T * ADE_GAIN_VRMS) + ADE_OFFS_VRMS;
    if (Vcal_R < ADE_VRMS_MIN) Vcal_R = 0.0f;
    if (Vcal_S < ADE_VRMS_MIN) Vcal_S = 0.0f;
    if (Vcal_T < ADE_VRMS_MIN) Vcal_T = 0.0f;

    /* ---- STEP 4: Kalibrasi arus — identik KWH.c ---- */
    float Rasio_CT = (float)ADE_CT_RATIO / 100.0f;
    float Ical_R = (Irms_R * ADE_GAIN_IRMS * Rasio_CT) + ADE_OFFS_IRMS;
    float Ical_S = (Irms_S * ADE_GAIN_IRMS * Rasio_CT) + ADE_OFFS_IRMS;
    float Ical_T = (Irms_T * ADE_GAIN_IRMS * Rasio_CT) + ADE_OFFS_IRMS;
    float Ical_N = (Irms_N * ADE_GAIN_IRMS * Rasio_CT) + ADE_OFFS_IRMS;

    /* Simpan raw counts & Ical sebelum filter untuk debugging
     * Catatan: AIRMS sudah di-remap ke Irms_T (fasa T), CIRMS ke Irms_R */
    ade_data.dbg_raw_AIRMS  = (int32_t)Irms_T;  /* AIRMS = CT fasa T */
    ade_data.dbg_raw_BIRMS  = (int32_t)Irms_S;  /* BIRMS = CT fasa S */
    ade_data.dbg_raw_CIRMS  = (int32_t)Irms_R;  /* CIRMS = CT fasa R */
    ade_data.dbg_Ical_T_raw = Ical_T;

    /* Noise floor suppression:
     * Channel dengan raw count ≤ ADE_IRMS_NOISE_CNT dianggap noise → paksa 0.
     * Threshold 1100 dipilih di atas noise floor F407 (~1019 counts).       */
    /* Filter 1: raw count di bawah noise floor → paksa 0 */
    if (Irms_R <= ADE_IRMS_NOISE_CNT) Ical_R = 0.0f;
    else if (Ical_R < 0.0f)           Ical_R = 0.0f;
    if (Irms_S <= ADE_IRMS_NOISE_CNT) Ical_S = 0.0f;
    else if (Ical_S < 0.0f)           Ical_S = 0.0f;
    if (Irms_T <= ADE_IRMS_NOISE_CNT) Ical_T = 0.0f;
    else if (Ical_T < 0.0f)           Ical_T = 0.0f;
    if (Irms_N <= ADE_IRMS_NOISE_CNT) Ical_N = 0.0f;
    else if (Ical_N < 0.0f)           Ical_N = 0.0f;

    /* Filter 2: nilai Ampere di bawah minimum valid → paksa 0
     * Menangkap noise yang lolos filter raw count (misal CIRMS=1189 > 1100) */
    if (Ical_R < ADE_ICAL_MIN) Ical_R = 0.0f;
    if (Ical_S < ADE_ICAL_MIN) Ical_S = 0.0f;
    if (Ical_T < ADE_ICAL_MIN) Ical_T = 0.0f;
    if (Ical_N < ADE_ICAL_MIN) Ical_N = 0.0f;

    /* ---- STEP 5: PF dan Frekuensi — identik ADE7880_getDataPFf() ---- */
    float pF_R, pF_S, pF_T, f_R, f_S, f_T;
    int   A_PF, B_PF, C_PF;
    int   FREQ_A, FREQ_B, FREQ_C;
    int16_t rd16;

    /* PF - R */
    ADE7880_CS_RESET(ADE7880_CS_PIN);
    ADE7880_Write8(0x01); ADE7880_Write16(APF);
    rd16 = (int16_t)ADE7880_Read16();
    ADE7880_CS_SET(ADE7880_CS_PIN);
    if (rd16 > 32768) rd16 = rd16 - 65536;
    A_PF = rd16;
    pF_R = fabsf(A_PF / 32768.0f);

    /* Frekuensi - R */
    ADE7880_CS_RESET(ADE7880_CS_PIN);
    ADE7880_Write8(0x01); ADE7880_Write16(APERIOD);
    rd16 = (int16_t)ADE7880_Read16();
    ADE7880_CS_SET(ADE7880_CS_PIN);
    FREQ_A = (int)(2560000 / rd16);
    f_R = FREQ_A / 10.0f;

    /* PF - S */
    ADE7880_CS_RESET(ADE7880_CS_PIN);
    ADE7880_Write8(0x01); ADE7880_Write16(BPF);
    rd16 = (int16_t)ADE7880_Read16();
    ADE7880_CS_SET(ADE7880_CS_PIN);
    if (rd16 > 32768) rd16 = rd16 - 65536;
    B_PF = rd16;
    pF_S = fabsf(B_PF / 32768.0f);

    /* Frekuensi - S */
    ADE7880_CS_RESET(ADE7880_CS_PIN);
    ADE7880_Write8(0x01); ADE7880_Write16(BPERIOD);
    rd16 = (int16_t)ADE7880_Read16();
    ADE7880_CS_SET(ADE7880_CS_PIN);
    FREQ_B = (int)(2560000 / rd16);
    f_S = FREQ_B / 10.0f;

    /* PF - T */
    ADE7880_CS_RESET(ADE7880_CS_PIN);
    ADE7880_Write8(0x01); ADE7880_Write16(CPF);
    rd16 = (int16_t)ADE7880_Read16();
    ADE7880_CS_SET(ADE7880_CS_PIN);
    if (rd16 > 32768) rd16 = rd16 - 65536;
    C_PF = rd16;
    pF_T = fabsf(C_PF / 32768.0f);

    /* Frekuensi - T */
    ADE7880_CS_RESET(ADE7880_CS_PIN);
    ADE7880_Write8(0x01); ADE7880_Write16(CPERIOD);
    rd16 = (int16_t)ADE7880_Read16();
    ADE7880_CS_SET(ADE7880_CS_PIN);
    FREQ_C = (int)(2560000 / rd16);
    f_T = FREQ_C / 10.0f;

    /* ---- STEP 6: Filter PF ---- */
    if (Vcal_R == 0.0f) pF_R = 0.0f;
    else if (pF_R < ADE_PF_MIN) pF_R = 0.0f;
    if (Vcal_S == 0.0f) pF_S = 0.0f;
    else if (pF_S < ADE_PF_MIN) pF_S = 0.0f;
    /* Channel T: V di CVRMS, I di AIRMS (beda channel) → ADE7880 tidak bisa
     * hitung CPF lintas channel. Gunakan PF = 1.0 (daya semu ≈ daya aktif
     * untuk beban resistif). Untuk beban non-resistif, kalibrasi manual. */
    if (Vcal_T > ADE_VRMS_MIN && Ical_T > 0.0f)
        pF_T = 1.0f;
    else
        pF_T = 0.0f;

    /* ---- STEP 7: Daya ---- */
    float P_R = Vcal_R * Ical_R * pF_R;
    float P_S = Vcal_S * Ical_S * pF_S;
    float P_T = Vcal_T * Ical_T * pF_T;   /* P = V × I (PF=1, apparent power) */

    /* ---- STEP 8: Baca register energi hardware ADE7880 (AWATTHR / CWATTHR) ----
     * ADE7880 mengakumulasi P×dt secara internal di DSP 8kHz (sangat akurat).
     * Register ini 32-bit signed, reset saat power off → perlu EEPROM untuk
     * penyimpanan permanen. Untuk sekarang: simpan di RAM + offset dari EEPROM.
     * AWATTHR → mapped ke fasa T (karena AIRMS = CT fasa T)                 */
    int32_t raw_whr_T, raw_whr_R, raw_whr_S;

    ADE7880_CS_RESET(ADE7880_CS_PIN);
    ADE7880_Delay(200);
    ADE7880_Write8(0x01); ADE7880_Write16(AWATTHR);
    raw_whr_T = (int32_t)ADE7880_Read32();   /* channel A = CT fasa T */
    ADE7880_CS_SET(ADE7880_CS_PIN);

    ADE7880_CS_RESET(ADE7880_CS_PIN);
    ADE7880_Delay(200);
    ADE7880_Write8(0x01); ADE7880_Write16(BWATTHR);
    raw_whr_S = (int32_t)ADE7880_Read32();
    ADE7880_CS_SET(ADE7880_CS_PIN);

    ADE7880_CS_RESET(ADE7880_CS_PIN);
    ADE7880_Delay(200);
    ADE7880_Write8(0x01); ADE7880_Write16(CWATTHR);
    raw_whr_R = (int32_t)ADE7880_Read32();   /* channel C = CT fasa R */
    ADE7880_CS_SET(ADE7880_CS_PIN);

    /* Register hardware AWATTHR/BWATTHR/CWATTHR dibaca tapi tidak dipakai
     * sebagai sumber WH utama — disimpan sebagai debug/referensi saja.      */
    (void)raw_whr_R;
    (void)raw_whr_S;
    (void)raw_whr_T;

    /* ── Akumulasi software — identik KWH_EC25_V1c: Wh += P / 3600 ──────────
     * Menggunakan dt aktual (bukan tetap 1 detik) untuk presisi lebih baik.
     * KWH_EC25_V1c pakai SysTick 1 detik tetap; kita pakai waktu konversi nyata
     * (~700ms per ADE7880_ReadAll). Keduanya valid, dt aktual lebih akurat.  */
    float elapsed_ms = (float)(HAL_GetTick() - t_start);
    float dt_hours   = elapsed_ms / 3600000.0f;
    Wh_R += P_R * dt_hours;
    Wh_S += P_S * dt_hours;
    Wh_T += P_T * dt_hours;

    /* ---- STEP 9: Simpan ke struct global ---- */
    ade_data.VRms_R = Vcal_R;  ade_data.VRms_S = Vcal_S;  ade_data.VRms_T = Vcal_T;
    ade_data.IRms_R = Ical_R;  ade_data.IRms_S = Ical_S;  ade_data.IRms_T = Ical_T;
    ade_data.IRms_N = Ical_N;
    ade_data.Pf_R   = pF_R;    ade_data.Pf_S   = pF_S;    ade_data.Pf_T   = pF_T;
    ade_data.Freq_R = f_R;     ade_data.Freq_S = f_S;     ade_data.Freq_T = f_T;
    ade_data.Pow_R  = P_R;     ade_data.Pow_S  = P_S;     ade_data.Pow_T  = P_T;
    ade_data.Pow_Total = P_R + P_S + P_T;
    /* WH software — sama seperti KWH_EC25_V1c */
    ade_data.WH_R     = Wh_R;
    ade_data.WH_S     = Wh_S;
    ade_data.WH_T     = Wh_T;
    ade_data.WH_Total = Wh_R + Wh_S + Wh_T;
    ade_data.conv_time_ms = HAL_GetTick() - t_start;
}

/* ============================================================
 * Reset akumulasi energi
 * ============================================================ */
void ADE7880_ResetWH(void)
{
    /* Reset variabel software — identik resetWH() di KWH_EC25_V1c */
    Wh_R = Wh_S = Wh_T = 0.0f;
    ade_data.WH_R = ade_data.WH_S = ade_data.WH_T = ade_data.WH_Total = 0.0f;
}




































