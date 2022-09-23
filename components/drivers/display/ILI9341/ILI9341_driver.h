#pragma once
#include "driver/spi_master.h"
#include "LVGL/lvgl.h"

/*******************************
 *      ILI9341 Commands
 * *****************************/

// System Function Command Table 1

// or use the constructor to over-ride defaults


#define TFT_RGB_BGR 0x08


  // COMMAND DEFINITION
  // ---------------------------------------------------------------
  #define ILI9341_NOP           0x00  // No Operation
  #define ILI9341_SWRESET       0x01  // Software Reset
  #define ILI9341_RDDIDIF       0x04  // Read Display Identification Information
  #define ILI9341_RDDST         0x09  // Read Display Status
  #define ILI9341_RDDPM         0x0A  // Read Display Power Mode
  #define ILI9341_RDDMADCTL     0x0B  // Read Display MADCTL
  #define ILI9341_RDDCOLMOD     0x0C  // Read Display Pixel Format
  #define ILI9341_RDDIM         0x0D  // Read Display Image Format
  #define ILI9341_RDDSM         0x0E  // Read Display Signal Mode
  #define ILI9341_RDDSDR        0x0F  // Read Display Self Diagnostics Result
  // ---------------------------------------------------------------
  #define ILI9341_SLPIN         0x10  // Enter Sleep Mode
  #define ILI9341_SLPOUT        0x11  // Sleep Out
  #define ILI9341_PTLON         0x12  // Partial Mode On
  #define ILI9341_NORON         0x13  // Normal Display On
  // ---------------------------------------------------------------
  #define ILI9341_DINVOFF       0x20  // Dislpay Inversion Off
  #define ILI9341_DINVON        0x21  // Dislpay Inversion On
  #define ILI9341_GAMSET        0x26  // Gamma Set  
  #define ILI9341_DISPOFF       0x28  // Display OFF
  #define ILI9341_DISPON        0x29  // Display ON
  #define ILI9341_CASET         0x2A  // Column Address Set
  #define ILI9341_RASET         0x2B  // Row Address Set
  #define ILI9341_RAMWR         0x2C  // Memory Write
  #define ILI9341_RGBSET        0x2D  // Color Set
  #define ILI9341_RAMRD         0x2E  // Memory Read
  // ---------------------------------------------------------------
  #define ILI9341_PLTAR         0x30  // Partial Area
  #define ILI9341_VSCRDEF       0x33  // Vertical Scroll Definition
  #define ILI9341_TEOFF         0x34  // Tearing Effect Line OFF
  #define ILI9341_TEON          0x35  // Tearing Effect Line ON
  #define ILI9341_MADCTL        0x36  // Memory Access Control
  #define ILI9341_VSSAD         0x37  // Vertical Scrolling Start Address
  #define ILI9341_IDMOFF        0x38  // Idle Mode OFF
  #define ILI9341_IDMON         0x39  // Idle Mode ON
  #define ILI9341_PIXFMT        0x3A  // Pixel Format Set
  #define ILI9341_WMCON         0x3C  // Write Memory Continue
  #define ILI9341_RMCON         0x3E  // Read Memory Continue
  // ---------------------------------------------------------------
  #define ILI9341_IFMODE        0xB0  // RGB Interface Signal Control
  #define ILI9341_FRMCRN1       0xB1  // Frame Control (In Normal Mode)
  #define ILI9341_FRMCRN2       0xB2  // Frame Control (In Idle Mode)
  #define ILI9341_FRMCRN3       0xB3  // Frame Control (In Partial Mode)
  #define ILI9341_INVTR         0xB4  // Display Inversion Control
  #define ILI9341_PRCTR         0xB5  // Blanking Porch Control
  #define ILI9341_DISCTRL       0xB6  // Display Function Control
  #define ILI9341_ETMOD         0xB7  // Entry Mode Set
  #define ILI9341_BKCR1         0xB8  // Backlight Control 1
  #define ILI9341_BKCR2         0xB9  // Backlight Control 2
  #define ILI9341_BKCR3         0xBA  // Backlight Control 3
  #define ILI9341_BKCR4         0xBB  // Backlight Control 4
  #define ILI9341_BKCR5         0xBC  // Backlight Control 5
  #define ILI9341_BKCR7         0xBE  // Backlight Control 7
  #define ILI9341_BKCR8         0xBF  // Backlight Control 8
  // ---------------------------------------------------------------
  #define ILI9341_PWCTRL1       0xC0  // Power Control 1
  #define ILI9341_PWCTRL2       0xC1  // Power Control 2
  #define ILI9341_VCCR1         0xC5  // VCOM Control 1
  #define ILI9341_VCCR2         0xC7  // VCOM Control 2
  #define ILI9341_LCD_POWERA    0xCB  // Power control A register
  #define ILI9341_LCD_POWERB    0xCF  // Power control B register
   // ---------------------------------------------------------------
  #define ILI9341_RDID1         0xDA  // Read ID1
  #define ILI9341_RDID2         0xDB  // Read ID2
  #define ILI9341_RDID3         0xDC  // Read ID3
  // ---------------------------------------------------------------
  #define ILI9341_GMCTRP1       0xE0  // Positive Gamma Correction
  #define ILI9341_GMCTRN1       0xE1  // Neagtove Gamma Correction
  #define ILI9341_LCD_DTCA      0xE8   // Driver timing control A
  #define ILI9341_LCD_DTCB      0xEA   // Driver timing control B
  #define ILI9341_LCD_POWER_SEQ 0xED   // Power on sequence register
  #define ILI9341_START         0xEF  // Mystery command  from ILItech
  // Extend register commands
  // --------------------------------------------------------------- 
  // @source https://github.com/fagcinsk/stm-ILI9341-spi/blob/master/lib/ILI9341/commands.h
  //
  #define ILI9341_LCD_3GAMMA_EN 0xF2   // 3 Gamma enable register
  #define ILI9341_INTFCTRL      0xF6   // Interface Control
  #define ILI9341_LCD_PRC       0xF7   // Pump ratio control register


/*******************************
 *      TYPEDEF
 * *****************************/
struct ILI9341_driver;

typedef struct ILI9341_transaction_data {
	struct ILI9341_driver *driver;
	bool data;
} ILI9341_transaction_data_t;

typedef uint16_t ILI9341_color_t;

typedef struct ILI9341_driver {
	int pin_reset;
	int pin_dc;
  int pin_cs;
	int pin_mosi;
	int pin_sclk;
	int spi_host;
	int dma_chan;
	uint8_t queue_fill;
	uint16_t display_width;
	uint16_t display_height;
	spi_device_handle_t spi;
	size_t buffer_size;
	ILI9341_transaction_data_t data;
	ILI9341_transaction_data_t command;
	ILI9341_color_t *buffer;
	ILI9341_color_t *buffer_primary;
	ILI9341_color_t *buffer_secondary;
	ILI9341_color_t *current_buffer;
	spi_transaction_t trans_a;
	spi_transaction_t trans_b;
} ILI9341_driver_t;

typedef struct ILI9341_command {
	uint8_t command;
	uint8_t wait_ms;
	uint8_t data_size;
	const uint8_t *data;
} ILI9341_command_t;


/*********************
 *      FUNCTIONS
 *********************/

/*
 * Function:  ILI9341_init 
 * --------------------
 * 
 * Initialize the SPI peripheral and send the initialization sequence.
 * 
 * Arguments:
 * 	-driver: Screen driver structure.
 * 
 * Returns: True if the initialization suceed otherwise false.
 * 
 */
bool ILI9341_init(ILI9341_driver_t *driver);

/*
 * Function:  ILI9341_reset
 * --------------------
 * 
 * Reset the display
 * 
 * Arguments:
 * 	-driver: Screen driver structure.
 * 
 * Returns: Nothing.
 * 
 */
void ILI9341_reset(ILI9341_driver_t *driver);

/*
 * Function:  ILI9341_fill_area
 * --------------------
 * 
 * Fill a area of the display with a selected color
 * 
 * Arguments:
 * 	-driver: Screen driver structure.
 * 	-color: 16 Bit hexadecimal color to fill the area.
 * 	-start_x: Start point on the X axis.
 * 	-start_y: Start point on the Y axis.
 * 	-width: Width of the area to be fill.
 * 	-height: Height of the area to be fill.
 * 
 * Returns: Nothing.
 * 
 */
void ILI9341_fill_area(ILI9341_driver_t *driver, ILI9341_color_t color, uint16_t start_x, uint16_t start_y, uint16_t width, uint16_t height);

/*
 * Function:  ILI9341_write_pixels 
 * --------------------
 * 
 * WIP
 * 
 * Arguments:
 * 	-driver: Screen driver structure.
 * 
 * Returns: Nothing.
 * 
 */
void ILI9341_write_pixels(ILI9341_driver_t *driver, ILI9341_color_t *pixels, size_t length);

/*
 * Function:  ILI9341_write_lines 
 * --------------------
 * 
 * WIP
 * 
 * Arguments:
 * 	-driver: Screen driver structure.
 * 
 * Returns: Nothing.
 * 
 */
void ILI9341_write_lines(ILI9341_driver_t *driver, int ypos, int xpos, int width, uint16_t *linedata, int lineCount);

/*
 * Function:  ILI9341_swap_buffers 
 * --------------------
 * 
 * The driver has two buffer, to allow send and render the image at the same type. This function
 * send the data of the actived buffer and change the pointer of current buffer to the next one.
 * 
 * Arguments:
 * 	-driver: Screen driver structure.
 * 
 * Returns: Nothing.
 * 
 */
void ILI9341_swap_buffers(ILI9341_driver_t *driver);

/*
 * Function:  ILI9341_set_window
 * --------------------
 * 
 * This screen allows partial update of the screen, so we can specified which part of the windows is going to change.
 * 
 * Arguments:
 * 	-driver: Screen driver structure.
 * 	-start_x: X axis start point of the refresh zone.
 * 	-start_y: Y axis start point of the refresh zone. 
 *	-end_x: X axis end point of the refresh zone. 
 *	-end_y: Y axis end point of the refresh zone. 

 * Returns: Nothing.
 * 
 */
void ILI9341_set_window(ILI9341_driver_t *driver, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y);

/*
 * Function:  ILI9341_set_endian 
 * --------------------
 * 
 * Depper explanation on the display_HAL.h file, but this function change the screen configuration from,
 * little endian message to big endian message.
 * 
 * Arguments:
 * 	-driver: Screen driver structure.
 * 
 * Returns: Nothing.
 * 
 */

void ILI9341_set_endian(ILI9341_driver_t *driver);