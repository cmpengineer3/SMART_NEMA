/*
 * ssd1306.h
 *
 *  Created on: Feb 11, 2026
 *      Author: firza
 */

#ifndef INC_SSD1306_H_
#define INC_SSD1306_H_
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>

#define SSD1309_I2C_ADDR        (0x3C << 1)
#define SSD1309_WIDTH           128
#define SSD1309_HEIGHT          64
#define SSD1309_PAGES           8
#define SSD1309_COLUMN_OFFSET   0
extern const unsigned char siklon_logo[];

/* ==================== FUNCTION PROTOTYPES ==================== */
HAL_StatusTypeDef SSD1309_Init(I2C_HandleTypeDef *hi2c);
void SSD1309_Clear(void);
void SSD1309_Fill(void);
void SSD1309_DisplayOn(void);
void SSD1309_DisplayOff(void);
void SSD1309_SetCursor(uint8_t column, uint8_t page);
void SSD1309_ClearPage(uint8_t page);
void SSD1309_ShowChar(uint8_t x, uint8_t page, char chr);
void SSD1309_ShowString(uint8_t x, uint8_t page, const char *str);
void SSD1309_ShowStringCenter(uint8_t page, const char *str);
void SSD1309_ShowCharScaled(uint8_t x, uint8_t page, char chr, uint8_t scale);
void SSD1309_ShowStringScaled(uint8_t x, uint8_t page, const char *str, uint8_t scale);
void SSD1309_DrawBitmap(const uint8_t *bitmap, uint8_t pages);
void SSD1309_ClearArea(uint8_t x, uint8_t page, uint8_t width);
void SSD1309_ClearRect(uint8_t x, uint8_t page, uint8_t width, uint8_t pages);
void SSD1309_DrawProgressBar(uint8_t x, uint8_t page, uint8_t width, uint8_t percent);


#endif /* INC_SSD1306_H_ */
