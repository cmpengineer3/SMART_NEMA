/*
 * HLW8110_Calc.c
 *
 *  Created on: Apr 10, 2026
 *      Author: firza
 */


/*
 * HLW8110_Calc.c
 *
 * Fungsi kalkulasi HLW8110/HLW8112
 * Sama persis dengan referensi asli
 */

#include "HLW8110_Calc.h"

/* ============================================================
 * KONSTANTA KALIBRASI
 * Ubah nilai ini sesuai dengan resistor di PCB kamu
 * ============================================================ */
float VK   = 0.34f;
float IK   = 5100.0f;
float Vdiv = 4194304.0f;        /* 2^22 */
float Idiv = 8388608.0f;        /* 2^23 */
float Pdiv = 2057483648.0f;
float Sdiv = 2097483648.0f;

/* Hasil kalkulasi semua channel */
parameter param;

/* Akses ke register buffer dari HLW8110_Access.c */
extern RegBuffer regbuffer;

/* ============================================================
 * FUNGSI KALKULASI
 * Formula sama persis dengan referensi asli
 * ============================================================ */

/**
 * Tegangan RMS
 * Formula: V = (Vreg × Vcoeff × 0.01) / (VK × Vdiv)
 */
float Val_Vrms(void)
{
    if (VK == 0 || Vdiv == 0) return 0.0f;
    return ((float)regbuffer.Vreg * (float)regbuffer.Vcoeff * 0.01f)
           / (VK * Vdiv);
}

/**
 * Arus RMS
 * Formula: I = (Ireg × Icoeff) / (IK × Idiv)
 */
float Val_Irms(void)
{
    if (IK == 0 || Idiv == 0) return 0.0f;
    return ((float)regbuffer.Ireg * (float)regbuffer.Icoeff)
           / (IK * Idiv);
}

/**
 * Daya Aktif
 * Formula: P = (Preg × Pcoeff × -1000) / (VK × IK × Pdiv)
 * Catatan: -1000 untuk koreksi tanda dan satuan (sama dengan referensi)
 */
float Val_Power(void)
{
    if (VK == 0 || IK == 0 || Pdiv == 0) return 0.0f;
    return ((float)((int32_t)regbuffer.Preg) * (float)regbuffer.Pcoeff * -1000.0f)
           / (VK * IK * Pdiv);
}

/**
 * Daya Semu
 * Formula: S = (Sreg × Scoeff × 1000) / (VK × IK × Sdiv)
 */
float Val_Apparent(void)
{
    if (VK == 0 || IK == 0 || Sdiv == 0) return 0.0f;
    return ((float)((int32_t)regbuffer.Sreg) * (float)regbuffer.Scoeff * 1000.0f)
           / (VK * IK * Sdiv);
}

/**
 * Power Factor channel ke-i
 * Formula: PF = ActivePower[i] / Apparent[i]
 * Sama persis dengan referensi
 */
float Val_PowerFactor(int i)
{
    if (param.Apparent[i] == 0.0f) return 0.0f;
    return param.ActivePower[i] / param.Apparent[i];
}

/**
 * Frekuensi
 * Formula: F = 3579545 / (Ufreqreg × 8)
 * Konstanta = frekuensi kristal internal HLW8110
 */
float Val_Frequency(void)
{
    if (regbuffer.Ufreqreg == 0) return 0.0f;
    return 3579545.0f / ((float)regbuffer.Ufreqreg * 8.0f);
}

/* ============================================================
 * ALIAS FUNGSI (kompatibilitas)
 * ============================================================ */
float ReadVoltage(void)   { return Val_Vrms(); }
float ReadCurrent(void)   { return Val_Irms(); }
float ReadPowerA(void)    { return Val_Power(); }
float ReadApparent(void)  { return Val_Apparent(); }
float ReadPF(uint8_t i)   { return Val_PowerFactor(i); }
float ReadFrequency(void) { return Val_Frequency(); }
