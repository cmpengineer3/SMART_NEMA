/*
 * fram.c — Driver FRAM/EEPROM via I2C (hi2c1)
 *
 * Catatan port:
 *  - Fungsi inti (Write/Read byte) disalin apa adanya dari KWH_EC25_V1c.
 *  - BUG DIPERBAIKI pada FRAM_Read_WH(): versi lama membaca ke variabel
 *    global 'mlatitude' lalu me-return 'WattH.m_float' yang belum diisi,
 *    sehingga selalu mengembalikan nilai sampah. Di sini dibaca & di-return
 *    dari union yang sama (WattH).
 */

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "i2c.h"
#include "fram.h"

/* Definisi global union WattH (dipakai juga di main.c untuk simpan WH) */
union float_n_byte WattH;

void WriteData_FRAM(int addr, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, addr, 64, &data, 1, 10);
    HAL_Delay(100);
}

uint8_t ReadData_FRAM(int addr)
{
    uint8_t data_read1;
    HAL_I2C_Mem_Read(&hi2c1, 0xA0, addr, 64, &data_read1, 1, 10);
    HAL_Delay(1);
    return data_read1;
}

void WriteChar_FRAM(int addr, char data_char[])
{
    int len = 0, j;
    len = strlen(data_char);
    for (j = 0; j <= len; j++)
    {
        uint8_t data_write = data_char[j];
        HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, addr + j, 64, &data_write, 1, 10);
        HAL_Delay(100);
    }
}

void ReadChar_FRAM(int addr0, int addrn, uint8_t data_read_char[])
{
    int i;
    for (i = 0; i <= (addrn - addr0); i++)
    {
        data_read_char[i] = ReadData_FRAM(addr0 + i);
    }
}

void WritemByte_FRAM(int addr0, uint8_t mByte[])
{
    int j;
    for (j = 0; j <= 4; j++)
    {
        uint8_t data_write = mByte[j];
        HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, addr0 + j, 64, &data_write, 1, 10);
        HAL_Delay(1);
    }
}

void ReadmByte_FRAM(int addr0, uint8_t mByte[])
{
    int j;
    for (j = 0; j <= 4; j++)
    {
        mByte[j] = ReadData_FRAM(addr0 + j);
    }
}

float Read_latitude(void)
{
    int j;
    union float_n_byte mlatitude;

    for (j = 0; j <= 4; j++)
    {
        mlatitude.m_bytes[j] = ReadData_FRAM(addr_latitude + j);
        HAL_Delay(1);
    }
    return mlatitude.m_float;
}

float FRAM_Read_WH(void)
{
    int j;
    union float_n_byte wh;

    for (j = 0; j <= 4; j++)
    {
        wh.m_bytes[j] = ReadData_FRAM(addr_energy + j);   /* FIX: baca ke 'wh' */
        HAL_Delay(1);
    }
    return wh.m_float;                                     /* FIX: return 'wh' */
}

float Read_longitude(void)
{
    int j;
    union float_n_byte mlongitude;

    for (j = 0; j <= 4; j++)
    {
        mlongitude.m_bytes[j] = ReadData_FRAM(addr_longitude + j);
        HAL_Delay(1);
    }
    return mlongitude.m_float;
}
