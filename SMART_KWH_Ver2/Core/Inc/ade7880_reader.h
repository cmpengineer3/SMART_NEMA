/**
 ******************************************************************************
 * @file    ade7880_reader.h
 * @brief   Driver ADE7880 berbasis kode referensi yang terbukti bekerja
 *          (powermeter_ade7880.c + KWH.c dari proyek asli)
 ******************************************************************************
 */
#ifndef ADE7880_READER_H
#define ADE7880_READER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* ===========================================================================
 * PIN CONFIGURATION — sesuaikan jika hardware berbeda
 * ===========================================================================*/
#define ADE7880_CS_PIN      GPIO_PIN_12
#define ADE7880_CS_PORT     GPIOB
#define ADE7880_SCK_PIN     GPIO_PIN_13
#define ADE7880_SCK_PORT    GPIOB
#define ADE7880_MOSI_PIN    GPIO_PIN_15
#define ADE7880_MOSI_PORT   GPIOB
#define ADE7880_MISO_PIN    GPIO_PIN_14
#define ADE7880_MISO_PORT   GPIOB
#define ADE7880_RESET_PIN   GPIO_PIN_10
#define ADE7880_RESET_PORT  GPIOD
#define ADE7880_PM0_PIN     GPIO_PIN_12
#define ADE7880_PM0_PORT    GPIOD
#define ADE7880_PM1_PIN     GPIO_PIN_11
#define ADE7880_PM1_PORT    GPIOD

/* GPIO macros — identik dengan referensi asli */
#define ADE7880_SCK_SET     HAL_GPIO_WritePin(ADE7880_SCK_PORT,  ADE7880_SCK_PIN,  GPIO_PIN_SET)
#define ADE7880_SCK_RESET   HAL_GPIO_WritePin(ADE7880_SCK_PORT,  ADE7880_SCK_PIN,  GPIO_PIN_RESET)
#define ADE7880_MOSI_SET    HAL_GPIO_WritePin(ADE7880_MOSI_PORT, ADE7880_MOSI_PIN, GPIO_PIN_SET)
#define ADE7880_MOSI_RESET  HAL_GPIO_WritePin(ADE7880_MOSI_PORT, ADE7880_MOSI_PIN, GPIO_PIN_RESET)
#define ADE7880_CS_SET(x)   HAL_GPIO_WritePin(ADE7880_CS_PORT,   x,                GPIO_PIN_SET)
#define ADE7880_CS_RESET(x) HAL_GPIO_WritePin(ADE7880_CS_PORT,   x,                GPIO_PIN_RESET)
#define ADE7880_MISO        HAL_GPIO_ReadPin (ADE7880_MISO_PORT, ADE7880_MISO_PIN)
#define ADE7880_RESET_HIGH  HAL_GPIO_WritePin(ADE7880_RESET_PORT,ADE7880_RESET_PIN,GPIO_PIN_SET)
#define ADE7880_RESET_LOW   HAL_GPIO_WritePin(ADE7880_RESET_PORT,ADE7880_RESET_PIN,GPIO_PIN_RESET)
#define ADE7880_Mode0_Set   { HAL_GPIO_WritePin(ADE7880_PM0_PORT,ADE7880_PM0_PIN,GPIO_PIN_SET); \
                              HAL_GPIO_WritePin(ADE7880_PM1_PORT,ADE7880_PM1_PIN,GPIO_PIN_RESET); }

/* ===========================================================================
 * KONFIGURASI PENGUKURAN
 * ===========================================================================*/
#define ADE_CT_RATIO        100     /* CT primer, default 100A/50mA */
#define ADE_AVG_SAMPLES     10      /* jumlah sample untuk averaging */
#define ADE_SAMPLE_DELAY_MS 50      /* delay antar sample (ms) — dari referensi */

/* ===========================================================================
 * KALIBRASI — dari referensi asli
 * ===========================================================================*/
#define ADE_GAIN_VRMS   0.0000891322683474717f
#define ADE_OFFS_VRMS   4.19731508227824f
/* Kalibrasi arus — dihitung dari pengukuran nyata hardware F407:
 * Titik kalibrasi: I=0.18A → raw_AIRMS=10623, noise floor=1016 (avg BIRMS+CIRMS)
 * GAIN = 0.18 / (10623 - 1016) = 0.18 / 9607 = 0.00001873648f
 * OFFS = -(1016 × GAIN) = -0.01904f                                             */
#define ADE_GAIN_IRMS   0.00001873648f
#define ADE_OFFS_IRMS   (-0.01904f)
#define ADE_IRMS_NOISE_CNT  1500    /* threshold noise raw — dinaikkan dari 1100 ke 1500
                                     * karena CIRMS tanpa CT terbaca ~1189 (noise floor).
                                     * Naikkan lagi jika masih bocor, turunkan jika
                                     * arus kecil yang valid terpotong.               */
#define ADE_ICAL_MIN        0.05f   /* batas minimum arus valid (Ampere) — arus di bawah
                                     * nilai ini dianggap noise dan dipaksa ke 0.
                                     * Sesuaikan dengan arus minimum beban nyata.     */
#define ADE_VRMS_MIN    120.0f      /* tegangan minimum valid (Volt) */
#define ADE_PF_MIN      0.10f       /* PF minimum valid */

/* ===========================================================================
 * REGISTER ADDRESS — dari powermeter_ade7880.h referensi
 * ===========================================================================*/
#define AIRMS   0x43C0
#define AVRMS   0x43C1
#define BIRMS   0x43C2
#define BVRMS   0x43C3
#define CIRMS   0x43C4
#define CVRMS   0x43C5
#define NIRMS   0x43C6
#define APF     0xE902
#define BPF     0xE903
#define CPF     0xE904
#define APERIOD 0xE905
#define BPERIOD 0xE906
#define CPERIOD 0xE907
#define RUN     0xE228
#define CONFIG  0xE618
/* Register akumulasi energi aktif (32-bit signed, reset saat power off) */
#define AWATTHR 0xE400
#define BWATTHR 0xE401
#define CWATTHR 0xE402
/* Konstanta konversi WATTHR → Wh
 * Dari datasheet: 1 LSB WATTHR = PMAX × FCLKIN / (4096 × WDIV × 3600 × 2^31)
 * Pendekatan praktis: gunakan konstanta kalibrasi dari pengukuran nyata.
 * Default sementara = 1.0 (perlu dikalibrasi dengan beban diketahui)        */
#define ADE_WATTHR_TO_WH    0.000001f   /* akan diupdate setelah kalibrasi */

/* ===========================================================================
 * DATA STRUCT — tambahkan "ade_data" ke Live Expression
 * ===========================================================================*/
typedef struct {
    float VRms_R, VRms_S, VRms_T;          /* Tegangan (Volt) */
    float IRms_R, IRms_S, IRms_T, IRms_N;  /* Arus (Ampere)   */
    float Pf_R,   Pf_S,   Pf_T;            /* Power Factor    */
    float Freq_R, Freq_S, Freq_T;          /* Frekuensi (Hz)  */
    float Pow_R,  Pow_S,  Pow_T, Pow_Total;/* Daya (Watt)     */
    float WH_R,   WH_S,   WH_T,  WH_Total; /* Energi (Wh)     */
    uint8_t  chip_status;                   /* 1=OK, 0=Error   */
    uint32_t conv_time_ms;                  /* durasi baca ms  */
    /* Debug: raw register ADE7880 sebelum kalibrasi — untuk diagnosis */
    int32_t  dbg_raw_AIRMS;                 /* raw AIRMS count (fasa R) */
    int32_t  dbg_raw_BIRMS;                 /* raw BIRMS count (fasa S) */
    int32_t  dbg_raw_CIRMS;                 /* raw CIRMS count (fasa T) */
    float    dbg_Ical_T_raw;               /* Ical_T sebelum filter PF */
} ADE7880_Data_t;

extern ADE7880_Data_t ade_data;

/* API */
void    ADE7880_Config(void);       /* init — panggil sekali di startup */
void    ADE7880_ReadAll(void);      /* baca semua — panggil periodik    */
void    ADE7880_ResetWH(void);      /* reset akumulasi energi           */

/* SPI primitives — public agar bisa dipakai dari luar jika perlu */
void     ADE7880_Write8(uint8_t out);
void     ADE7880_Write16(uint16_t out);
uint16_t ADE7880_Read16(void);
uint32_t ADE7880_Read32(void);

#ifdef __cplusplus
}
#endif
#endif
