#include "ssd1306.h"

#define SSD1306_TX_BUF_DISABLE  0   	// SSD1306 do not need buffer
#define SSD1306_RX_BUF_DISABLE	0   	// SSD1306 do not need buffer
#define SSD1306_FREQ_HZ    		100000  // SSD1306 I2C clock frequency
#define ACK_CHECK_DIS  			0x0     // I2C master will not check ack from slave

/* SSD1306 data buffer */
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

/* Private SSD1306 structure */
typedef struct {
	int16_t CurrentX;
	int16_t CurrentY;
	uint8_t Inverted;
	i2c_port_t i2c_num;
	uint8_t Initialized;
	uint8_t* buffer;
	SSD1306_COLOR_t color;
} SSD1306_t;

/* Private variable */
static SSD1306_t SSD1306;

/* Private low level functions */
static void SSD1306_Connect(uint8_t _sda, uint8_t _scl) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = SSD1306_FREQ_HZ;
    i2c_param_config(SSD1306.i2c_num, &conf);
    i2c_driver_install(SSD1306.i2c_num, conf.mode,
    				   SSD1306_RX_BUF_DISABLE,
					   SSD1306_TX_BUF_DISABLE, 0);
}
static esp_err_t SSD1306_I2CWrite(uint8_t* data_wr, size_t size) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SSD1306_I2C_ADDR << 1 ), ACK_CHECK_DIS);
  i2c_master_write(cmd, data_wr, size, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(SSD1306.i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}
static esp_err_t SSD1306_I2CWriteMulti(uint8_t addr, uint32_t count, uint8_t* data_wr) {
  uint8_t I2CBuffer[SSD1306_BUFFER_SIZE + 1];
  I2CBuffer[0] = addr;
  memcpy(&I2CBuffer[1], data_wr, count);
  esp_err_t ret = SSD1306_I2CWrite(I2CBuffer, count+1);
  return ret;
}
esp_err_t SSD1306_WriteCommand(uint8_t command){
	uint8_t I2CBuffer[2];
	I2CBuffer[0] = 0x0;
	I2CBuffer[1] = command;
	esp_err_t ret = SSD1306_I2CWrite(I2CBuffer, 0x2);
	return ret;
}

uint8_t SSD1306_Init(i2c_port_t port, uint8_t _sda, uint8_t _scl) {

	/* Init LCD */
	SSD1306.i2c_num = port;
	SSD1306_Connect(_sda, _scl);

	SSD1306_WriteCommand(DISPLAYOFF);
	SSD1306_SetMemoryMode(HORIZONTAL);			//Horizontal Addressing Mode
	SSD1306_WriteCommand(COMSCANINC);
	SSD1306_WriteCommand(SETSTARTLINE);
	SSD1306_SetColumnAddress(0x0,0x7F);
	SSD1306_SetPageAddress(0x0,0x7);
	SSD1306_SetContrast(0xCF);					// set contrast to CF
	SSD1306_WriteCommand(SEGREMAP);
	SSD1306_WriteCommand(NORMALDISPLAY);
	SSD1306_WriteCommand(SETMULTIPLEX);
	SSD1306_WriteCommand(0x3F);
	SSD1306_WriteCommand(DISPLAYALLON_RESUME);
	SSD1306_WriteCommand(SETDISPLAYOFFSET);
	SSD1306_WriteCommand(0x00);
	SSD1306_SetClockDivideRatio(0);
	SSD1306_WriteCommand(SETPRECHARGE);
	SSD1306_WriteCommand(0xF1);
	SSD1306_WriteCommand(SETCOMPINS);
	SSD1306_WriteCommand(0x12);
	SSD1306_WriteCommand(0xDB);
	SSD1306_WriteCommand(0x20);
	SSD1306_WriteCommand(CHARGEPUMP);
	SSD1306_WriteCommand(0x14);
	SSD1306_WriteCommand(0x2e); // stop scroll
	SSD1306_WriteCommand(DISPLAYON);

	SSD1306.buffer = (uint8_t*) malloc(sizeof(uint8_t) * SSD1306_BUFFER_SIZE);

	/* Clear screen */
	SSD1306_Fill(SSD1306_COLOR_BLACK);

	/* Update screen */
	SSD1306_UpdateScreen();

	/* Set default values */
	SSD1306.CurrentX = 0;
	SSD1306.CurrentY = 0;

	/* Initialized OK */
	SSD1306.Initialized = 1;

	/* Return OK */
	return 1;
}

void SSD1306_setColor(SSD1306_COLOR_t color) {
	SSD1306.color = color;
}

void SSD1306_UpdateScreen(void) {
	int ret;
	SSD1306_SetPageAddress(0x00,0x07);
	SSD1306_SetColumnAddress(0x00,0x7F);
	ret = SSD1306_I2CWriteMulti(0x40, SSD1306_BUFFER_SIZE, SSD1306_Buffer);
    if (ret == ESP_FAIL) {
        printf("I2C Fail\n");
    }
}

void SSD1306_Fill(SSD1306_COLOR_t color) {
	/* Set memory */
	memset(SSD1306_Buffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));
	memset(SSD1306.buffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, SSD1306_BUFFER_SIZE*sizeof(uint8_t));
}

void SSD1306_DrawPixel(int16_t x, int16_t y) {
	if (x >= 0 && x < 128 && y >= 0 && y < 64) {
		switch (SSD1306.color) {
			case SSD1306_COLOR_WHITE:
				SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |=  (1 << (y & 7));
				break;
			case SSD1306_COLOR_BLACK:
				SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y & 7));
				break;
			case SSD1306_COLOR_INVERSE:
				SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] ^=  (1 << (y & 7));
				break;
		}
	}
}

void SSD1306_SetContrast(uint8_t contrast)
{
	SSD1306_WriteCommand(SETCONTRAST);
	SSD1306_WriteCommand(contrast);
}

void SSD1306_SetClockDivideRatio(uint8_t ratio)
{
	SSD1306_WriteCommand(SETDISPLAYCLOCKDIV);
	SSD1306_WriteCommand(0xF0 | (ratio & 0x0F));
}

void SSD1306_SetPreCharge(uint8_t Phase1, uint8_t Phase2)
{
	SSD1306_WriteCommand(SETPRECHARGE);
	SSD1306_WriteCommand(((Phase2 & 0x0F)<<4) | (Phase1 & 0x0F));
}

void SSD1306_SetMemoryMode(uint8_t mode)
{
	SSD1306_WriteCommand(MEMORYMODE); 			//Set Memory Addressing Mode
	SSD1306_WriteCommand(mode); 				//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode
}

void SSD1306_SetColumnAddress(uint8_t start, uint8_t end)
{
	SSD1306_WriteCommand(COLUMNADDR);
	SSD1306_WriteCommand(start);
	SSD1306_WriteCommand(end);
}

void SSD1306_SetPageAddress(uint8_t start, uint8_t end)
{
	SSD1306_WriteCommand(PAGEADDR);
	SSD1306_WriteCommand(start);
	SSD1306_WriteCommand(end);
}
