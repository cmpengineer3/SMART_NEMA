/*
 * HLW8110_Calc.h
 *
 * Fungsi kalkulasi HLW8110/HLW8112
 * Mengkonversi raw register → nilai fisik (V, A, W, VA, PF, Hz)
 */

#ifndef INC_HLW8110_CALC_H_
#define INC_HLW8110_CALC_H_

#include "stm32g4xx_hal.h"
#include "stdint.h"
#include "stdbool.h"
#include "HLW8110_Access.h"

/* ============================================================
 * JUMLAH CHANNEL
 * ============================================================ */
#define HLW_CHANNEL_COUNT   3

/* ============================================================
 * KONSTANTA KALIBRASI
 * Sesuaikan dengan nilai resistor di PCB
 * ============================================================ */
extern float VK;        /* Voltage divider factor  (default: 0.34)      */
extern float IK;        /* Current shunt factor    (default: 5100.0)     */
extern float Vdiv;      /* = 2^22 = 4194304.0                            */
extern float Idiv;      /* = 2^23 = 8388608.0                            */
extern float Pdiv;      /* = 2057483648.0                                */
extern float Sdiv;      /* = 2097483648.0                                */

/* ============================================================
 * STRUCT HASIL KALKULASI
 * Bisa dipantau di Live Expression:
 *   param.RMSU[0]       → Tegangan fasa R (V)
 *   param.RMSU[1]       → Tegangan fasa S (V)
 *   param.RMSU[2]       → Tegangan fasa T (V)
 *   param.RMSIA[0]      → Arus fasa R (A)
 *   param.ActivePower[0]→ Daya aktif fasa R (W)
 *   param.Apparent[0]   → Daya semu fasa R (VA)
 *   param.PowerFactor[0]→ Power factor fasa R
 *   param.Frequency[0]  → Frekuensi fasa R (Hz)
 * ============================================================ */
typedef struct {
    float RMSU[HLW_CHANNEL_COUNT];          /* Tegangan RMS (V) */
    float RMSIA[HLW_CHANNEL_COUNT];         /* Arus RMS (A) */
    float ActivePower[HLW_CHANNEL_COUNT];   /* Daya Aktif (W) */
    float Apparent[HLW_CHANNEL_COUNT];      /* Daya Semu (VA) */
    float PowerFactor[HLW_CHANNEL_COUNT];   /* Power Factor (0.0 - 1.0) */
    float Frequency[HLW_CHANNEL_COUNT];     /* Frekuensi (Hz) */
} parameter;

extern parameter param;

/* ============================================================
 * DEKLARASI FUNGSI KALKULASI
 * Panggil setelah HLW_Request() + HAL_Delay()
 * ============================================================ */

/** Hitung tegangan RMS dari regbuffer saat ini → Volt */
float Val_Vrms(void);

/** Hitung arus RMS dari regbuffer saat ini → Ampere */
float Val_Irms(void);

/** Hitung daya aktif dari regbuffer saat ini → Watt */
float Val_Power(void);

/** Hitung daya semu dari regbuffer saat ini → VA */
float Val_Apparent(void);

/**
 * @brief Hitung power factor channel ke-i
 * @param i  Index channel (0=R, 1=S, 2=T)
 * @return   Power factor (0.0 - 1.0)
 */
float Val_PowerFactor(int i);

/** Hitung frekuensi dari regbuffer saat ini → Hz */
float Val_Frequency(void);

/* Alias fungsi (kompatibilitas) */
float ReadVoltage(void);
float ReadCurrent(void);
float ReadPowerA(void);
float ReadApparent(void);
float ReadPF(uint8_t i);
float ReadFrequency(void);

#endif /* INC_HLW8110_CALC_H_ */
