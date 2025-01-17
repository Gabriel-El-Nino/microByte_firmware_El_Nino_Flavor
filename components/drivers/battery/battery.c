/*********************
*      INCLUDES
*********************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <driver/adc.h>
#include "esp_log.h"
#include "esp_adc_cal.h"

#include "battery.h"
#include "system_manager.h"

#include "math.h"

/**********************
 *      VARIABLES
 **********************/

struct BATTERY_STATUS battery_status;

bool game_mode_active = false; // Variable to know if we're playing to avoid queue overflow
bool battery_alert = false; // To avoid system block when the low battery raise we need a control if the alert was done.

/**********************
 *      STATIC 
 **********************/

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_0;   
static const adc_atten_t atten = ADC_ATTEN_11db; //1dB of attenuation to obtain the full voltage range of the GPIO (Up to 3.9V)
static const adc_unit_t unit = ADC_UNIT_1;

static const char *TAG = "Battery";

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void battery_init(void){
    // Initialization of ADC0_1 to measure battery level

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel,atten);

    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, 1040, adc_chars);

    xTaskCreatePinnedToCore(&batteryTask, "Battery management", 2048, NULL, 5, NULL, 0);
    
}

void battery_game_mode(bool game_mode){
    game_mode_active = game_mode;
}

uint8_t battery_get_percentage(){
    return battery_status.percentage;
}

void batteryTask(void *arg){
    while(1){

        uint32_t adc_reading = 0;

        //Perform a multisampling reading of the ADC1 Channel 0
        for(int i = 0; i < 128; i++){
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        }

        adc_reading /= 128;
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

        voltage += 1310; //Add 1000 mV to offset the value obtained from the voltage divider
        battery_status.voltage = voltage;

        //To avoid lost of precision, first we calculate the percentage in float.
        float percentage_aux = (voltage-3570.6)/5.2128;
        //Explanation of this weird thing:
        // The curve from this point (3576 mV) is not lineal so, to avoid split the curve and
        // add a new polinomic function I've follow the ugly/lazy path, it's 1% all the time.
        if(battery_status.voltage < 3576)  percentage_aux = 1.00;
        battery_status.percentage = (uint8_t)percentage_aux;


        //When you're charging the battery, the percentage can higher than 100%
        if(battery_status.percentage > 100) battery_status.percentage = 100;

        // if(!game_mode_active){
        //     // If we're playing, we don't need the battery info. We only need the an alert if we're on very low percentage
        //     if( xQueueSend( batteryQueue,&battery_status, ( TickType_t ) 10) != pdPASS ){
        //         ESP_LOGE(TAG,"Battery queue send fail");
        //     }
        // }
        
        ESP_LOGI(TAG, "Battery voltage in mAh: %d", battery_status.voltage);
        ESP_LOGI(TAG, "Battery percentage: %d", battery_status.percentage);

        if(battery_status.percentage<=10 && battery_alert != true){
            //Send battery alert if the level is below 10%
            struct SYSTEM_MODE management;
            management.mode = MODE_BATTERY_ALERT;

            if( xQueueSend( modeQueue,&management, ( TickType_t ) 10) != pdPASS ){
                ESP_LOGE(TAG,"Mode queue send fail, battery alert!");
            }

            battery_alert = true; //The alert was done, it's not necessay to do it again 
        }
        else if(battery_status.percentage>10 && battery_alert == true) battery_alert = false;

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}