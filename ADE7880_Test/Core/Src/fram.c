#include <stdio.h>
#include <string.h>
#include "main.h"
#include "i2c.h"
#include "fram.h"

#define FRAM_MEMADD_SIZE   I2C_MEMADD_SIZE_16BIT

/* Timeout tiap transaksi I2C (ms) */
#define FRAM_I2C_TIMEOUT   100

/* Global union WattH (dipakai juga di main.c untuk simpan WH) */
union float_n_byte WattH;

void WriteData_FRAM(int addr, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, addr, FRAM_MEMADD_SIZE,
                      &data, 1, FRAM_I2C_TIMEOUT);
    HAL_Delay(1);
}

uint8_t ReadData_FRAM(int addr)
{
    uint8_t data_read1 = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, addr, FRAM_MEMADD_SIZE,
                         &data_read1, 1, FRAM_I2C_TIMEOUT) != HAL_OK)
    {
        return 0;   /* gagal -> kembalikan 0, jangan hang */
    }
    return data_read1;
}

void WriteChar_FRAM(int addr, char data_char[])
{
    int len = (int)strlen(data_char);
    for (int j = 0; j <= len; j++)
    {
        uint8_t data_write = (uint8_t)data_char[j];
        HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, addr + j, FRAM_MEMADD_SIZE,
                          &data_write, 1, FRAM_I2C_TIMEOUT);
        HAL_Delay(1);
    }
}

void ReadChar_FRAM(int addr0, int addrn, uint8_t data_read_char[])
{
    for (int i = 0; i <= (addrn - addr0); i++)
        data_read_char[i] = ReadData_FRAM(addr0 + i);
}

void WritemByte_FRAM(int addr0, uint8_t mByte[])
{
    for (int j = 0; j < 4; j++)   /* float = 4 byte (referensi j<=4 = overflow) */
    {
        HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, addr0 + j, FRAM_MEMADD_SIZE,
                          &mByte[j], 1, FRAM_I2C_TIMEOUT);
        HAL_Delay(1);
    }
}

void ReadmByte_FRAM(int addr0, uint8_t mByte[])
{
    for (int j = 0; j < 4; j++)
        mByte[j] = ReadData_FRAM(addr0 + j);
}

float Read_latitude(void)
{
    union float_n_byte v;
    for (int j = 0; j < 4; j++)
        v.m_bytes[j] = ReadData_FRAM(addr_latitude + j);
    return v.m_float;
}

float Read_longitude(void)
{
    union float_n_byte v;
    for (int j = 0; j < 4; j++)
        v.m_bytes[j] = ReadData_FRAM(addr_longitude + j);
    return v.m_float;
}

float FRAM_Read_WH(void)
{
    union float_n_byte wh;
    for (int j = 0; j < 4; j++)              /* FIX: j<4, bukan j<=4 */
        wh.m_bytes[j] = ReadData_FRAM(addr_energy + j);
    return wh.m_float;
}
