#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "system_configuration.h"

#include "GUI.h"
#include "display_HAL.h"
#include "GUI_frontend.h"

#include "LVGL/lvgl.h"

#include "esp_log.h"

/*********************
 *      DEFINES
 *********************/
#define LV_TICK_PERIOD_MS 10
#define DISP_BUF_SIZE  SCR_WIDTH * 20 // Horizontal Res * 40 vetical pixels

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_tick_task(void *arg);
static lv_disp_drv_t disp_drv;

/**********************
 *  STATIC VARIABLES
 **********************/
static SemaphoreHandle_t xGuiSemaphore;
static const char *TAG = "GUI.c";

/**********************
*   GLOBAL FUNCTIONS
**********************/

void GUI_init(void){

    xGuiSemaphore = xSemaphoreCreateMutex();

    // LVGL Initialization
    lv_init();
    ESP_LOGI(TAG, "lv initialised");

    //Screen Buffer initialization
    static EXT_RAM_ATTR lv_color_t * buf1[DISP_BUF_SIZE];
   // static EXT_RAM_ATTR lv_color_t * buf2[DISP_BUF_SIZE];

    static lv_disp_buf_t disp_buf;
    uint32_t size_in_px = DISP_BUF_SIZE; 

    lv_disp_buf_init(&disp_buf, buf1, NULL, size_in_px);

    // Initialize LVGL display and attach the flush function
    
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = display_HAL_flush;

    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    // Create timer to handle LVGL system tick
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &lv_tick_task,
            .name = "periodic_gui"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    //On ESP32 it's better to create a periodic task instead of esp_register_freertos_tick_hook
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000)); // LV_TICK_PERIOD_MS expressed as microseconds

    // Init menu graphic user interface
    GUI_frontend();
    ESP_LOGI(TAG, "GUI_frontend initialised");

}


// Async refresh
void GUI_refresh(){
    for(int i=0;i<3;i++) lv_disp_flush_ready(&disp_drv);
}

void GUI_async_message(){
    async_battery_alert();
}

void GUI_task(void *arg){

    GUI_init();
    while (1) {
        lv_task_handler();
    }
    printf("delete\r\n");
    //A task should NEVER return
    vTaskDelete(NULL);
}

/**********************
*   STATIC FUNCTIONS
**********************/

static void lv_tick_task(void *arg) {
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}

