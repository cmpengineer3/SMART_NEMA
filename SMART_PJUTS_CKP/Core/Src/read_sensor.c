/*
 * read_srne.c
 *
 *  Created on: Sep 4, 2025
 *      Author: firza
 */
#include "read_sensor.h"
#include "config.h"
#include "mqtt.h"
#include "uart.h"   // untuk huart3
extern UART_HandleTypeDef huart3;

/* Modbus_CRC16 dipakai bersama dengan CRC payload MQTT — implementasinya
 * ada di mqtt.c, prototype-nya di mqtt.h (ter-include lewat config.h).
 * Tidak didefinisikan ulang di sini agar tidak "multiple definition". */

// ==================== Read Device & Battery Temperature ====================
void Read_Temperatures(void)
{
    uint8_t request[] = {0x01, 0x03, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00};
    uint16_t crc = Modbus_CRC16(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;

    uint8_t response[7];

    HAL_UART_Transmit(&huart3, request, sizeof(request), 10000);
    if (HAL_UART_Receive(&huart3, response, sizeof(response), 10000) == HAL_OK)
    {
        uint16_t crc_calc = Modbus_CRC16(response, 5);
        uint16_t crc_recv = response[5] | (response[6] << 8);
        if (crc_calc == crc_recv)
        {
            uint8_t devRaw = response[3];
            uint8_t batRaw = response[4];

            int8_t devSign = (devRaw & 0x80) ? -1 : 1;
            int8_t batSign = (batRaw & 0x80) ? -1 : 1;

            deviceTemperature = devSign * (devRaw & 0x7F);
            batteryTemperature = batSign * (batRaw & 0x7F);
        }
    }
}


// ===============================Charge State =================================================
void Read_Charge_State(void) {
    uint8_t request[8];
    uint8_t response[7];
    uint16_t crc;

    request[0] = 0x01; // Device ID
    request[1] = 0x03; // Function code
    request[2] = 0x00; // High byte
    request[3] = 0xFD; // Register 0x00FD
    request[4] = 0x00;
    request[5] = 0x01; // 1 register

    crc = Modbus_CRC16(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;

    HAL_UART_Transmit(&huart3, request, 8, 10000);

    if (HAL_UART_Receive(&huart3, response, 7, 500) == HAL_OK) {
        uint16_t crc_calc = Modbus_CRC16(response, 5);
        uint16_t crc_recv = response[5] | (response[6] << 8);

        if (crc_calc == crc_recv) {
            uint16_t raw = (response[3] << 8) | response[4];

            uint8_t high = (raw >> 8) & 0xFF;
            uint8_t low  = raw & 0xFF;

            loadState     = (high & 0x80) ? 1 : 0; // b7
            loadIntensity = high & 0x7F;           // b0-b6
            chargeState   = low;                   // langsung mapping
        }
    }
}
//====================================== VERSI 2 ModBus ============================================================
// Fungsi umum untuk baca 1 register Modbus
uint8_t Read_Modbus_Register(uint16_t regAddr, float scale, float *result) {
    uint8_t request[8];
    uint8_t response[16];
    uint16_t crc;

    request[0] = 0x01;                     // Address device SRNE
    request[1] = 0x03;                     // Function code
    request[2] = (regAddr >> 8) & 0xFF;    // Start address high
    request[3] = regAddr & 0xFF;           // Start address low
    request[4] = 0x00;                     // Number of registers high
    request[5] = 0x01;                     // Number of registers low

    crc = Modbus_CRC16(request, 6);
    request[6] = crc & 0xFF;        // CRC Low
    request[7] = (crc >> 8) & 0xFF; // CRC High

    // Flush RX buffer supaya tidak tercampur data lama
//    __HAL_UART_FLUSH_DRREGISTER(&huart3);
//    memset(response, 0, sizeof(response));

    // Kirim request
    HAL_UART_Transmit(&huart3, request, 8, 10000);

    // Terima response
    if (HAL_UART_Receive(&huart3, response, 7, 500) == HAL_OK) {
        uint16_t crc_calc = Modbus_CRC16(response, 5);
        uint16_t crc_recv = response[5] | (response[6] << 8);

        if (crc_calc == crc_recv) {
            uint16_t raw_value = (response[3] << 8) | response[4];
            *result = raw_value * scale;
            return 1; // sukses
        }
    }
    return 0; // gagal


//    // Terima response (7 byte seharusnya)
//    int len = HAL_UART_Receive(&huart3, response, 7, 300); // timeout 300 ms
//    if (len != 7) {
//        // Length tidak sesuai → jangan update nilai
//        return 0; // gagal
//    }
//
//    // CRC check
//    uint16_t crc_calc = Modbus_CRC16(response, 5);
//    uint16_t crc_recv = response[5] | (response[6] << 8);
//
//    if (crc_calc != crc_recv) {
//        // CRC salah → jangan update nilai
//        return 0; // gagal
//    }
//
//    // Kalau valid → update nilai
//    uint16_t raw_value = (response[3] << 8) | response[4];
//    *result = raw_value * scale;
//    return 1; // sukses
}

// ----------------- Fungsi spesifik -------------------
// ----------------- Battery Related -------------------
void Read_Battery_Voltage(void) {
    Read_Modbus_Register(0x0101, 0.1f, &batteryVoltage);
}

void Read_Battery_Current(void){
	Read_Modbus_Register(0x0102, 0.01f, &batteryCurrent);
}

void Read_Battery_SOC(void) {
    Read_Modbus_Register(0x0100, 1.0f, &batterySOC);
}

// ----------------- Load Related -------------------
void Read_Load_Voltage(void) {
    Read_Modbus_Register(0x0104, 0.1f, &loadVoltage);
}

void Read_Load_Current(void) {
    Read_Modbus_Register(0x0105, 0.01f, &loadCurrent);
}

void Read_Load_Power(void) {
    Read_Modbus_Register(0x0106, 1.0f, &loadPower);
}

// ----------------- PV Related -------------------
void Read_PV_Voltage(void) {
    Read_Modbus_Register(0x0107, 0.1f, &pvVolt);
}

void Read_PV_Current(void) {
    Read_Modbus_Register(0x0108, 0.01f, &pvCurrent);
}

void Read_PV_Power(void) {
	Read_Modbus_Register(0x0109, 1.0f, &pvPower);
}

void Read_All_Sensor(void)
{
    // Battery
    Read_Battery_Voltage();
    HAL_Delay(20);
    Read_Battery_Current();
    HAL_Delay(20);
    Read_Battery_SOC();
    HAL_Delay(20);

    // Load
    Read_Load_Voltage();
    HAL_Delay(20);
    Read_Load_Current();
    HAL_Delay(20);
    Read_Load_Power();
    HAL_Delay(20);
    // PV
    Read_PV_Voltage();
    HAL_Delay(20);
    Read_PV_Current();
    HAL_Delay(20);
    Read_PV_Power();
    HAL_Delay(20);

    // Temperature
    Read_Temperatures();
    HAL_Delay(20);

    // Charging State
    Read_Charge_State();
    HAL_Delay(20);

    // MQTT
    mqtt_read_time();
    HAL_Delay(20);
//    mqtt_read_gps();
//    HAL_Delay(20);
}
