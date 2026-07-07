/*
 * read_srne.h
 *
 *  Created on: Sep 4, 2025
 *      Author: firza
 */

#ifndef INC_READ_SENSOR_H_
#define INC_READ_SENSOR_H_

#include <stdint.h>   // untuk uint8_t, uint16_t, dll.
#include <stddef.h>

/* Modbus_CRC16 dideklarasikan di mqtt.h (dipakai bersama untuk CRC MQTT
 * payload dan CRC Modbus RTU). Tidak dideklarasikan ulang di sini. */
extern void Read_All_Sensor(void);
extern void Read_Temperatures(void);
extern void Read_Charge_State(void);
extern uint8_t Read_Modbus_Register(uint16_t regAddr, float scale, float *result);
extern void Read_Battery_Voltage(void);
extern void Read_Battery_Current(void);
extern void Read_Battery_SOC(void);
// ----------------- Load Related -------------------
extern void Read_Load_Voltage(void);
extern void Read_Load_Current(void);
extern void Read_Load_Power(void);
// ----------------- PV Related -------------------
extern void Read_PV_Voltage(void);
extern void Read_PV_Current(void);
extern void Read_PV_Power(void);
#endif /* INC_READ_SENSOR_H_ */
