#ifndef SSD1306_H
#define SSD1306_H

#include "stdlib.h"
#include "string.h"
#include "esp_system.h"
#include "driver/i2c.h"

/* I2C address */
#define SSD1306_I2C_ADDR         0x3C

/* SSD1306 settings */
#define SSD1306_WIDTH      	 	 128	// SSD1306 width in pixels
#define SSD1306_HEIGHT  	  	  64	// SSD1306 LCD height in pixels
#define SSD1306_BUFFER_SIZE		1024

// Display commands
#define CHARGEPUMP 0x8D
#define COLUMNADDR 0x21
#define COMSCANDEC 0xC8
#define COMSCANINC 0xC0
#define DISPLAYALLON 0xA5
#define DISPLAYALLON_RESUME 0xA4
#define DISPLAYOFF 0xAE
#define DISPLAYON 0xAF
#define EXTERNALVCC 0x1
#define INVERTDISPLAY 0xA7
#define MEMORYMODE 0x20
#define NORMALDISPLAY 0xA6
#define PAGEADDR 0x22
#define SEGREMAP 0xA0
#define SETCOMPINS 0xDA
#define SETCONTRAST 0x81
#define SETDISPLAYCLOCKDIV 0xD5
#define SETDISPLAYOFFSET 0xD3
#define SETHIGHCOLUMN 0x10
#define SETLOWCOLUMN 0x00
#define SETMULTIPLEX 0xA8
#define SETPRECHARGE 0xD9
#define SETSEGMENTREMAP 0xA1
#define SETSTARTLINE 0x40
#define HORIZONTAL	0x00
#define VERTICAL	0x01
#define PAGE		0X10
#define SETVCOMDETECT 0xDB
#define SWITCHCAPVCC 0x2

/**
 * @brief  SSD1306 color enumeration
 */

typedef enum {
	SSD1306_COLOR_BLACK = 0x00,		// Black color, no pixel
	SSD1306_COLOR_WHITE = 0x01,  	// Pixel is set. Color depends on LCD
	SSD1306_COLOR_INVERSE = 0x02
} SSD1306_COLOR_t;

esp_err_t SSD1306_WriteCommand(uint8_t command);

void SSD1306_setColor(SSD1306_COLOR_t color);

/**
 * @brief  Initializes SSD1306
 * @param  None
 * @retval Initialization status:
 *           - 0: SSD1306 was not detected on I2C port
 *           - >0: SSD1306 initialized OK and ready to use
 */
uint8_t SSD1306_Init(i2c_port_t port, uint8_t _sda, uint8_t _scl);

/**
 * @brief  Updates buffer from internal RAM to LCD
 * @note   This function must be called each time you do some changes to LCD, to update buffer from RAM to LCD
 * @param  None
 * @retval None
 */
void SSD1306_UpdateScreen(void);

/**
 * @brief  Toggles pixels invertion inside internal RAM
 * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  None
 * @retval None
 */

void SSD1306_Fill(SSD1306_COLOR_t Color);

/**
 * @brief  Draws pixel at desired location
 * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x: X location. This parameter can be a value between 0 and SSD1306_WIDTH - 1
 * @param  y: Y location. This parameter can be a value between 0 and SSD1306_HEIGHT - 1
 * @param  color: Color to be used for screen fill. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
 * @retval None
 */
void SSD1306_DrawPixel(int16_t x, int16_t y);

/**
 * @brief  Sets cursor pointer to desired location for strings
 * @param  x: X location. This parameter can be a value between 0 and SSD1306_WIDTH - 1
 * @param  y: Y location. This parameter can be a value between 0 and SSD1306_HEIGHT - 1
 * @retval None
 */

void SSD1306_SetContrast(uint8_t contrast);
void SSD1306_SetClockDivideRatio(uint8_t ratio);
void SSD1306_SetPreCharge(uint8_t Phase1 , uint8_t Phase2);
void SSD1306_SetMemoryMode(uint8_t mode);
void SSD1306_SetColumnAddress(uint8_t start, uint8_t end);
void SSD1306_SetPageAddress(uint8_t start, uint8_t end);

#endif
