#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "logger.h"

#define LOG_HOST "http://192.168.88.187:5000/log"
#define RAW_SAMPLES_HOST "http://192.168.88.187:5000/samples"
#define RAW_CHUNK_SIZE 1000

QueueHandle_t logger_queue;
esp_http_client_config_t json_config = {.url = LOG_HOST};
esp_http_client_handle_t json_client;
esp_http_client_config_t samples_config = {.url = RAW_SAMPLES_HOST};
esp_http_client_handle_t samples_client;
esp_err_t err;

void logger_task(void* arg)
{
    event_log_t entry;
    while(1)
    {
        if (xQueueReceive(logger_queue, &entry, portMAX_DELAY) == pdPASS)
        {
            ESP_LOGI("LOGGER", "Logging event on server");
            char json_buf[128];
            snprintf(json_buf, sizeof(json_buf),"{\"cat\":\"%s\",\"weight\":\"%d\",\"type\":\"%d\",\"amount\":\"%d\",\"duration\":\"%d\"}",
                                            entry.cat_id, entry.cat_weight, entry.type, entry.amount, entry.duration_sec);
            json_client = esp_http_client_init(&json_config);
            while (1)
            {
                esp_http_client_set_method(json_client, HTTP_METHOD_POST);
                esp_http_client_set_header(json_client, "Content-Type", "application/json");
                esp_http_client_set_post_field(json_client, json_buf, strlen(json_buf));
                err = esp_http_client_perform(json_client);
                if (err == ESP_OK)
                {
                    ESP_LOGI("LOGGER", "POST successful: %d", esp_http_client_get_status_code(json_client));
                    break;
                }
                else
                {
                    ESP_LOGE("LOGGER", "POST failed: %s, retrying", esp_err_to_name(err));
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }
            esp_http_client_cleanup(json_client);
        
            if (entry.rawDate != NULL)
            {
                ESP_LOGI("LOGGER", "Sending raw data file to server...");
                samples_client = esp_http_client_init(&samples_config);
                esp_http_client_set_method(samples_client, HTTP_METHOD_POST);
                esp_http_client_set_header(samples_client, "Content-Type", "application/octet-stream");
                while(1)
                {
                    err = esp_http_client_open(samples_client, entry.samples_cnt * sizeof(uint32_t));
                    if (err == ESP_OK)
                        break;
                    else
                    {
                        ESP_LOGE("LOGGER", "Failed to open connection for raw samples upload: %s, retrying...", esp_err_to_name(err));
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                }

                int chunk_len = 0;
                for (uint32_t i = 0; i < entry.samples_cnt; i += RAW_CHUNK_SIZE)
                {
                    chunk_len = (RAW_CHUNK_SIZE < entry.samples_cnt - i) ?  (RAW_CHUNK_SIZE * sizeof(uint32_t)) : ((entry.samples_cnt - i) * sizeof(uint32_t));
                    while (1)
                    {
                        if (esp_http_client_write(samples_client, (char*)entry.rawDate + i * sizeof(uint32_t), chunk_len) == chunk_len)
                            break;
                        else
                        {
                            ESP_LOGE("LOGGER", "Failed to send chunk, retrying...");
                            vTaskDelay(pdMS_TO_TICKS(1000));
                        }
                    }
                }
                free(entry.rawDate);
                ESP_LOGI("LOGGER", "Raw data file sent to server");
                esp_http_client_close(samples_client);
                esp_http_client_cleanup(samples_client);
            }
        }
    }
}

void logger_init(void)
{
    ESP_LOGI("LOGGER", "Starting logger...");
    logger_queue = xQueueCreate(3, sizeof(event_log_t));
    xTaskCreate(logger_task, "logger_task", 4096, NULL, 5, NULL);
    ESP_LOGI("LOGGER", "Logger task created");
}
