/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/param.h>

#include "ILI9341_driver.h"
#include "system_configuration.h"
#include "LVGL/lvgl.h"

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

/*********************
 *      DEFINES
 *********************/
#define ILI9341_SPI_QUEUE_SIZE 2

/**********************
*      VARIABLES
**********************/
static const char *TAG = "ILI9341_driver";

/**********************
*  STATIC PROTOTYPES
**********************/
static void ILI9341_send_cmd(ILI9341_driver_t *driver, const ILI9341_command_t *command);
static void ILI9341_config(ILI9341_driver_t *driver);
static void ILI9341_pre_cb(spi_transaction_t *transaction);
static void ILI9341_queue_empty(ILI9341_driver_t *driver);
static void ILI9341_multi_cmd(ILI9341_driver_t *driver, const ILI9341_command_t *sequence);


/**********************
 *   GLOBAL FUNCTIONS
 **********************/
bool ILI9341_init(ILI9341_driver_t *driver){
    //Allocate the buffer memory
    driver->buffer = (ILI9341_color_t *)heap_caps_malloc(driver->buffer_size * 2 * sizeof(ILI9341_color_t), MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    if(driver->buffer == NULL){
        ESP_LOGE(TAG, "Display buffer allocation fail");
        return false;
    }

    ESP_LOGI(TAG,"Display buffer allocated with a size of: %i",driver->buffer_size * 2 * sizeof(ILI9341_color_t));

    // Why set buffer, primary and secondary instead,just primary and secondary??
    //Set-up the display buffers
    driver->buffer_primary =  driver->buffer;
    driver->buffer_secondary = driver->buffer + driver->buffer_size;
    driver->current_buffer = driver->buffer_primary;
    driver->queue_fill = 0;

    driver->data.driver = driver;
	driver->data.data = true;
	driver->command.driver = driver;
	driver->command.data = false;

    // Set the RESET, DC and CS PIN
    gpio_pad_select_gpio(HSPI_RST);
    gpio_pad_select_gpio(HSPI_DC);
    gpio_pad_select_gpio(HSPI_CS);
    gpio_set_direction(HSPI_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(HSPI_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(HSPI_CS, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG,"Set RST pin: %i \n Set DC pin: %i \n Set CS pin: %i",HSPI_RST,HSPI_DC,HSPI_CS);

    // Set-Up SPI BUS
    spi_bus_config_t buscfg = {
		.mosi_io_num    = HSPI_MOSI,
		.miso_io_num    = -1,
		.sclk_io_num    = HSPI_CLK,
		.quadwp_io_num  = -1,
		.quadhd_io_num  = -1,
		.max_transfer_sz= driver->buffer_size * 2 * sizeof(ILI9341_color_t), // 2 buffers with 2 bytes for pixel
		.flags          = SPICOMMON_BUSFLAG_NATIVE_PINS
	};

    // Configure SPI BUS
    spi_device_interface_config_t devcfg = {
		.clock_speed_hz = HSPI_CLK_SPEED,
		.mode           = 3,
		.spics_io_num   = HSPI_CS,
		.queue_size     = ILI9341_SPI_QUEUE_SIZE,
		.pre_cb         = ILI9341_pre_cb,
	};

    if(spi_bus_initialize(HSPI_HOST, &buscfg, 1) != ESP_OK){
        ESP_LOGE(TAG,"SPI Bus initialization failed.");
        return false;
    }

    if(spi_bus_add_device(HSPI_HOST, &devcfg,&driver->spi) != ESP_OK){
        ESP_LOGE(TAG,"SPI Bus add device failed.");
        return false;
    }

    ESP_LOGI(TAG,"SPI Bus configured correctly.");

    // Set the screen configuration
    ILI9341_reset(driver);
    ILI9341_config(driver);

    ESP_LOGI(TAG,"Display configured and ready to work.");

    
    return true;
}


void ILI9341_reset(ILI9341_driver_t *driver) {
	gpio_set_level(HSPI_RST, 0);
	vTaskDelay(20 / portTICK_PERIOD_MS);
	gpio_set_level(HSPI_RST, 1);
	vTaskDelay(130 / portTICK_PERIOD_MS);
}


void ILI9341_fill_area(ILI9341_driver_t *driver, ILI9341_color_t color, uint16_t start_x, uint16_t start_y, uint16_t width, uint16_t height){
    // Fill the buffer with the selected color
	for (size_t i = 0; i < driver->buffer_size * 2; ++i) {
		driver->buffer[i] = color;
	}

    // Set the working area on the screen
	ILI9341_set_window(driver, start_x, start_y, start_x + width - 1, start_y + height - 1);

	size_t bytes_to_write = width * height * 2;
	size_t transfer_size = driver->buffer_size * 2 * sizeof(ILI9341_color_t);

	spi_transaction_t trans;
    spi_transaction_t *rtrans;

	memset(&trans, 0, sizeof(trans));
	trans.tx_buffer = driver->buffer;
	trans.user = &driver->data;
	trans.length = transfer_size * 8;
	trans.rxlength = 0;

	
	while (bytes_to_write > 0) {
		if (driver->queue_fill >= ILI9341_SPI_QUEUE_SIZE) {
			spi_device_get_trans_result(driver->spi, &rtrans, portMAX_DELAY);
			driver->queue_fill--;
		}
		if (bytes_to_write < transfer_size) {
			transfer_size = bytes_to_write;
		}
		spi_device_queue_trans(driver->spi, &trans, portMAX_DELAY);
		driver->queue_fill++;
		bytes_to_write -= transfer_size;
	}

	ILI9341_queue_empty(driver);
}

void ILI9341_write_pixels(ILI9341_driver_t *driver, ILI9341_color_t *pixels, size_t length){
	ILI9341_queue_empty(driver);

	spi_transaction_t *trans = driver->current_buffer == driver->buffer_primary ? &driver->trans_a : &driver->trans_b;
	memset(trans, 0, sizeof(&trans));
	trans->tx_buffer = driver->current_buffer;
	trans->user = &driver->data;
	trans->length = length * sizeof(ILI9341_color_t) * 8;
	trans->rxlength = 0;

	spi_device_queue_trans(driver->spi, trans, portMAX_DELAY);
	driver->queue_fill++;
}

void ILI9341_write_lines(ILI9341_driver_t *driver, int ypos, int xpos, int width, uint16_t *linedata, int lineCount){
    driver->buffer_size = width*20 ; 
	ILI9341_set_window(driver,xpos,ypos,width+xpos-1,ypos + 20);
   	ILI9341_swap_buffers(driver);
}

void ILI9341_swap_buffers(ILI9341_driver_t *driver){
	ILI9341_write_pixels(driver, driver->current_buffer, driver->buffer_size);
	driver->current_buffer = driver->current_buffer == driver->buffer_primary ? driver->buffer_secondary : driver->buffer_primary;
}

void ILI9341_set_window(ILI9341_driver_t *driver, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y){
	uint8_t caset[4];
	uint8_t raset[4];
    
	caset[0] = (uint8_t)(start_x >> 8) & 0xFF;
	caset[1] = (uint8_t)(start_x & 0xff);
	caset[2] = (uint8_t)(end_x >> 8) & 0xFF;
	caset[3] = (uint8_t)(end_x & 0xff) ;
	raset[0] = (uint8_t)(start_y >> 8) & 0xFF;
	raset[1] = (uint8_t)(start_y & 0xff);
	raset[2] = (uint8_t)(end_y >> 8) & 0xFF;
	raset[3] = (uint8_t)(end_y & 0xff);

	ILI9341_command_t sequence[] = {
		{ILI9341_CASET, 0, 4, caset},
		{ILI9341_RASET, 0, 4, raset},
		{ILI9341_RAMWR, 0, 0, NULL},
		{ILI9341_NOP, 0, 0, NULL},
	};

	ILI9341_multi_cmd(driver, sequence);
}

void ILI9341_set_endian(ILI9341_driver_t *driver){
	const ILI9341_command_t init_sequence_endian[] = {
		{ILI9341_INTFCTRL, 0, 3, (const uint8_t *)"\x00\x00\x00"},
	};
	ILI9341_send_cmd(driver, init_sequence_endian);
}
/**********************
 *   STATIC FUNCTIONS
 **********************/

static void ILI9341_pre_cb(spi_transaction_t *transaction) {
	const ILI9341_transaction_data_t *data = (ILI9341_transaction_data_t *)transaction->user;
	gpio_set_level(HSPI_DC, data->data);
}


static void ILI9341_config(ILI9341_driver_t *driver){

    const uint8_t caset[4] = {
		0x00,
		0x00,
		(driver->display_width - 1) >> 8,
		(driver->display_width - 1) & 0xff
	};
	const uint8_t raset[4] = {
		0x00,
		0x00,
		(driver->display_height - 1) >> 8,
		(driver->display_height - 1) & 0xff
	};
    
	const ILI9341_command_t init_sequence[] = {
        {ILI9341_START,         0, 3,  (const uint8_t *)"\x03\x80\x02"},
        {ILI9341_LCD_POWERB,    0, 3,  (const uint8_t *)"\x00\xC1\x30"},
        {ILI9341_LCD_POWER_SEQ, 0, 4,  (const uint8_t *)"\x64\x03\x12\x81"},
        {ILI9341_LCD_DTCA,      0, 3,  (const uint8_t *)"\x85\x00\x78"},
        {ILI9341_LCD_POWERA,    0, 5,  (const uint8_t *)"\x39\x2C\x00\x34\x02"},
        {ILI9341_LCD_PRC,       0, 1,  (const uint8_t *)"\x20"},
        {ILI9341_LCD_DTCB,      0, 2,  (const uint8_t *)"\x00\x00"},
        {ILI9341_PWCTRL1,       0, 1,  (const uint8_t *)"\x23"},          //VRH[5:0]
        {ILI9341_PWCTRL2,       0, 1,  (const uint8_t *)"\x10"},          //SAP[2:0];BT[3:0]
        {ILI9341_VCCR1,         0, 2,  (const uint8_t *)"\x3e\x28"},
        {ILI9341_VCCR2,         0, 1,  (const uint8_t *)"\x86"},
        {ILI9341_MADCTL,        0, 1,  (const uint8_t *)"\xE8"},          // Orientation and RGB inversion with current setup 
        {ILI9341_PIXFMT,        0, 1,  (const uint8_t *)"\x55"},
        {ILI9341_FRMCRN1,       0, 2,  (const uint8_t *)"\x00\x1B"},      // 0x18 79Hz, 0x1B default 70Hz, 0x13 100Hz
        {ILI9341_DISCTRL,       0, 3,  (const uint8_t *)"\x08\x82\x27"},
        {ILI9341_LCD_3GAMMA_EN, 0, 1,  (const uint8_t *)"\x00"},
        {ILI9341_GAMSET,        0, 1,  (const uint8_t *)"\x01"},
        {ILI9341_GMCTRP1,       0, 15, (const uint8_t *)"\x0F\x31\x2B\x0C\x0E\x08\x4E\xF1\x37\x07\x10\x03\x0E\x09\x00"},
        {ILI9341_GMCTRN1,       0, 15, (const uint8_t *)"\x00\x0E\x14\x03\x11\x07\x31\xC1\x48\x08\x0F\x0C\x31\x36\x0F"},
		{ILI9341_SLPOUT,      120,  0,  NULL},           // Sleep out
		{ILI9341_DISPON,      120,  0,  NULL},           //Display on
		{ILI9341_NOP, 0, 0, NULL},                      // End of commands
	};


	ILI9341_multi_cmd(driver, init_sequence);
	ILI9341_fill_area(driver, 0x0000, 0, 0, driver->display_width, driver->display_height);
	
	const ILI9341_command_t init_sequence_RAMWR[] = {
		{ILI9341_CASET, 0, 4, (const uint8_t *)&caset},
	    {ILI9341_RASET, 0, 4, (const uint8_t *)&raset},
		{ILI9341_RAMWR, 0, 0, NULL},
		{ILI9341_NOP, 0, 0, NULL},                      // End of commands
	};
	ILI9341_multi_cmd(driver, init_sequence_RAMWR);
}

static void ILI9341_send_cmd(ILI9341_driver_t *driver, const ILI9341_command_t *command){
    spi_transaction_t *return_trans;
	spi_transaction_t data_trans;
    
    // Check if the SPI queue is empty
    ILI9341_queue_empty(driver);

    // Send the command
	memset(&data_trans, 0, sizeof(data_trans));
	data_trans.length = 8; // 8 bits
	data_trans.tx_buffer = &command->command;
	data_trans.user = &driver->command;

	spi_device_queue_trans(driver->spi, &data_trans, portMAX_DELAY);
	spi_device_get_trans_result(driver->spi, &return_trans, portMAX_DELAY);

    // Send the data if the command has.
	if (command->data_size > 0) {
		memset(&data_trans, 0, sizeof(data_trans));
		data_trans.length = command->data_size * 8;
		data_trans.tx_buffer = command->data;
		data_trans.user = &driver->data;

		spi_device_queue_trans(driver->spi, &data_trans, portMAX_DELAY);
		spi_device_get_trans_result(driver->spi, &return_trans, portMAX_DELAY);
	}

    // Wait the required time
	if (command->wait_ms > 0) {
		vTaskDelay(command->wait_ms / portTICK_PERIOD_MS);
	}
    //ESP_LOGI(TAG, "ILI9341_send_cmd done");
}

static void ILI9341_multi_cmd(ILI9341_driver_t *driver, const ILI9341_command_t *sequence){
    while (sequence->command != ILI9341_NOP) {
		ILI9341_send_cmd(driver, sequence);
		sequence++;
	}
}

static void ILI9341_queue_empty(ILI9341_driver_t *driver){
	spi_transaction_t *return_trans;

	while (driver->queue_fill > 0) {
		spi_device_get_trans_result(driver->spi, &return_trans, portMAX_DELAY);
		driver->queue_fill--;
	}
}

