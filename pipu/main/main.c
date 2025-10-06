#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "wifi_station.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "logger.h"

const char *TAG = "app_main";

// QueueHandle_t logger_queue;

void test_task(void* arg)
{
    while(1)
    {
        event_log_t entry;
        entry.cat_id = "test";
        entry.amount = 100;
        entry.type = 1;
        entry.cat_weight = 50123;
        entry.samples_cnt = 0;
        entry.rawDate = NULL;
        entry.duration_sec = 35;
        xQueueSend(logger_queue, &entry,0);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

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
    
    wifi_station_init();
    ESP_LOGI(TAG, "Finished wifi_station_init , calling logger_init");
    
    logger_init();
    ESP_LOGI(TAG, "Finished logger_init , calling sampler_init");

    xTaskCreate(test_task, "test_task", 4096, NULL, 5, NULL);
    
    // sampler_init();
    // ESP_LOGI(TAG, "Finished sampler_init");
    // ESP_LOGI(TAG, "App initialzation done!");

}