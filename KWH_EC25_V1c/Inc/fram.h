#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "i2c.h"

#define EEPROM_ADDRESS 0xA0

#define addr_check 0
#define addr_energy 10
#define addr_latitude 40
#define addr_longitude 44
#define addr_sn 50
#define addr_jam_pwm_1 50

extern void WriteData_FRAM(int addr, uint8_t data);
extern uint8_t ReadData_FRAM(int addr);
extern void WriteChar_FRAM(int addr, char data_char[]);
extern void ReadChar_FRAM(int addr0, int addrn, uint8_t data_read_char[]);
extern void WritemByte_FRAM(int addr0, uint8_t mByte[]);
extern void ReadmByte_FRAM(int addr0, uint8_t mByte[]);
extern float Read_latitude(void);
extern float Read_longitude(void);
extern float FRAM_Read_WH(void);

#ifdef __cplusplus
}
#endif

