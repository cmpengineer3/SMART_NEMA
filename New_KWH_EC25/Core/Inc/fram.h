/*
 * fram.h — Driver FRAM/EEPROM via I2C (hi2c1)
 *
 * Dipakai untuk menyimpan nilai WH (energi akumulatif) secara persisten,
 * sehingga nilai kWh tidak hilang saat perangkat mati/restart.
 *
 * Dependency: i2c.h (hi2c1 harus sudah di-init via MX_I2C1_Init).
 */

#ifndef INC_FRAM_H_
#define INC_FRAM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "i2c.h"

#define EEPROM_ADDRESS 0xA0

/* ── Peta alamat FRAM ─────────────────────────────────────────────────────── */
#define addr_check      0
#define addr_energy     10
#define addr_latitude   40
#define addr_longitude  44
#define addr_sn         50
#define addr_jam_pwm_1  50

/* Union untuk konversi float <-> 4 byte (untuk simpan/baca WH ke FRAM) */
union float_n_byte {
    float    m_float;
    uint8_t  m_bytes[sizeof(float)];
};

/* WattH dideklarasikan global agar bisa dipakai bersama di main.c
 * (definisi sesungguhnya ada di fram.c). */
extern union float_n_byte WattH;

/* ── API ──────────────────────────────────────────────────────────────────── */
void    WriteData_FRAM(int addr, uint8_t data);
uint8_t ReadData_FRAM(int addr);
void    WriteChar_FRAM(int addr, char data_char[]);
void    ReadChar_FRAM(int addr0, int addrn, uint8_t data_read_char[]);
void    WritemByte_FRAM(int addr0, uint8_t mByte[]);
void    ReadmByte_FRAM(int addr0, uint8_t mByte[]);
float   Read_latitude(void);
float   Read_longitude(void);
float   FRAM_Read_WH(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_FRAM_H_ */
