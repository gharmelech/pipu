#ifndef LOGGER_H
#define LOGGER_H


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern QueueHandle_t logger_queue;
void logger_init(void);

typedef struct
{
    char cat_id[16];
    uint16_t cat_weight;
    uint8_t type;
    uint8_t amount;
    uint8_t duration_sec;
    void    *rawDate; 
    uint32_t samples_cnt;
}event_log_t;

#endif //LOGGER_H
