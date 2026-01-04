#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "wifi_station.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "logger.h"
#include "sampler.h"

const char *TAG = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting app main");
    ESP_LOGI(TAG, "Init nvs...");
    //init nvs
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Finished nvs init, calling wifi_station_init");

    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // wifi_station_init();
    // ESP_LOGI(TAG, "Finished wifi_station_init , calling logger_init");
    
    logger_init();
    ESP_LOGI(TAG, "Finished logger_init , calling sampler_init");
    
    sampler_init();
    ESP_LOGI(TAG, "Finished sampler_init");
    
    ESP_LOGI(TAG, "App initialzation done!");

}