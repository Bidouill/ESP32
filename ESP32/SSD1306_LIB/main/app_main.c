#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <nvs.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "ssd1306.h"

#define TAG "SSD1306_Demo"

#define SSD1306_SCL_IO    4    		// gpio number for I2C master clock
#define SSD1306_SDA_IO    5    		// gpio number for I2C master data
#define SSD1306_NUM 	  I2C_NUM_1 // I2C port number for master dev


/**
 * entry point
 */
void app_main()
{
	nvs_flash_init();
    SSD1306_Init(SSD1306_NUM, SSD1306_SDA_IO, SSD1306_SCL_IO);
    SSD1306_setColor(SSD1306_COLOR_WHITE);
    SSD1306_DrawPixel(SSD1306_WIDTH/2, SSD1306_HEIGHT/2);
	SSD1306_UpdateScreen();
	while (1);
}
