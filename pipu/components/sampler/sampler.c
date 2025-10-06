#include <stdio.h>
#include "sampler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "logger.h"
#include "driver/uart.h"

#define CONVERTION_FACTOR 1
#define SPS 80
#define SEC_FOR_AVG 10
#define AVG_SAMPLE_CNT (SEC_FOR_AVG * SPS)
#define CAT_THRESH_G 1000
#define BOX_THRESH_G 5000
#define LID_THRESH_G 100
#define NO_CHANGE_THRESH_G 10
#define DONE_CLEANING_STABLE_SEC_THRESH 10
#define POO_DETECTION_THRESH_G 2
#define DONE_CLEANING_STABLE_SEC_MAX 240
#define MAX_DEPOSIT_LEN_S 300
#define MAX_SAMPLES_BETWEEN_TRANSIENT_PARTS 10
#define SETTLING_TIME_S 5
#define NUM_OF_CAT_IDS_TO_BUFF 3
#define SIZE_OF_CAT_ID_UART_RX (16 * 4)
#define NUM_OF_UART_COMMANDS_TO_BUFF 3
#define SIZE_OF_UART_COMMAND_RX 8

QueueHandle_t tag_queue;
uint16_t offset = 0;
state_t state = idle;

typedef enum
{
    idle,
    cleaning,
    deposit
} state_t;

void sampler_fsm(void)
{
    int32_t sample_raw = 0;
    uint16_t sample_g = 0;
    uint16_t prev_sample_g = 0;
    uint32_t one_sec_sample_g = 0;
    uint16_t prev_sec_sample_g = 0;
    uint16_t measurement_delta = 0;
    uint16_t min_g = 0;
    uint16_t box_weight = 0;
    uint16_t weight_during_deposit = 0;
    uint16_t cnt = 0;
    bool possible_lid = false;
    uint16_t first_half_poo_transient = 0;
    bool cat_inside = false;
    bool poo_detected = false;
    uint8_t stable_sec_cnt = 0;
    char cat_id[16]; //15 chars + string termination
    int32_t *samples_buffer;
    while (1)
    {
        sample_raw = take_sample();//take_sample waits for interupt, returns summed and averaged sample from all sensors
        sample_g = (sample_raw * CONVERTION_FACTOR) - offset;
        switch (state)
        {
            case idle:
                if (sample_g > (box_weight + CAT_THRESH_G))
                {
                    scan_tag(cat_id);
                    samples_buffer =(int32_t*)malloc(sizeof(int32_t) * SPS * MAX_DEPOSIT_LEN_S);
                    cnt = 0;
                    one_sec_sample_g = 0;
                    prev_sec_sample_g = 0;
                    prev_sample_g = sample_g;
                    first_half_poo_transient = 0;
                    stable_sec_cnt = 0;
                    poo_detected = false;
                    cat_inside = false;
                    weight_during_deposit = sample_g;
                    state = deposit;
                }
                else if (sample_g < (box_weight - BOX_THRESH_G))
                {
                    cnt = 0;
                    min_g = sample_g;
                    one_sec_sample_g = 0;
                    prev_sec_sample_g = 0;
                    stable_sec_cnt = 0;
                    state = cleaning;
                }
                break;
            case cleaning:
                //avg over 1 sec, compare to previous sec
                one_sec_sample_g = one_sec_sample_g + sample_g;
                if (cnt < SPS)
                {
                    cnt++;
                    if (sample_g < min_g)
                        min_g = sample_g;
                }
                else
                {
                    cnt = 0;
                    one_sec_sample_g = one_sec_sample_g / SPS;
                    measurement_delta = abs(prev_sec_sample_g - one_sec_sample_g);
                    prev_sec_sample_g = one_sec_sample_g;
                    one_sec_sample_g = 0;
                    if ((measurement_delta > LID_THRESH_G) && (measurement_delta < CAT_THRESH_G))
                    {
                        possible_lid = true;
                        stable_sec_cnt = 0;
                    }
                    else if (measurement_delta < NO_CHANGE_THRESH_G) //cleaning done
                    {
                        stable_sec_cnt++;
                        if ((possible_lid && (stable_sec_cnt >= DONE_CLEANING_STABLE_SEC_THRESH)) || (stable_sec_cnt >= DONE_CLEANING_STABLE_SEC_MAX))
                        {
                            offset = offset + min_g;
                            box_weight = prev_sec_sample_g;
                            event_log_t entry;
                            entry.cat_id = "0";
                            entry.amount = 0;
                            entry.type = 0;
                            entry.cat_weight = box_weight;
                            entry.samples_cnt = 0;
                            entry.rawDate = NULL;
                            entry.duration_sec = 0;
                            xQueueSend(logger_queue, &entry,0);
                            possible_lid = false;
                            state = idle;
                        }
                    }
                    else
                    {
                        possible_lid = false;
                        stable_sec_cnt = 0;
                    }
                }
                break;
            case deposit:
                samples_buffer[cnt] = sample_raw;
                one_sec_sample_g = one_sec_sample_g + sample_g;
                if (cat_inside && !poo_detected)
                {
                    if (first_half_poo_transient)
                    {
                        if (sample_g > weight_during_deposit + POO_DETECTION_THRESH_G)
                            {
                                poo_detected = true;
                                first_half_poo_transient = 0;
                            }
                        else
                            first_half_poo_transient--;
                    }
                    else if ((prev_sample_g - sample_g >= 1) && (prev_sample_g - sample_g < NO_CHANGE_THRESH_G)) // very small drop in weight
                        first_half_poo_transient = MAX_SAMPLES_BETWEEN_TRANSIENT_PARTS;
                }
                prev_sample_g = sample_g;

                if ((cnt / SPS > 0) && (cnt % SPS == 0)) // completed one second of samples
                {
                    one_sec_sample_g = one_sec_sample_g / SPS;
                    measurement_delta = one_sec_sample_g - prev_sec_sample_g;
                    prev_sec_sample_g = one_sec_sample_g;
                    one_sec_sample_g = 0;
                    if (measurement_delta < NO_CHANGE_THRESH_G)
                    {
                        stable_sec_cnt++;
                        if (stable_sec_cnt >= SETTLING_TIME_S)
                            if (!cat_inside && (prev_sec_sample_g - box_weight > CAT_THRESH_G))
                            {
                                cat_inside = true;
                                weight_during_deposit = prev_sec_sample_g;
                            }
                            else if (weight_during_deposit - prev_sec_sample_g > CAT_THRESH_G)
                            {
                                //cat left, summerize deposit
                                cat_inside = false;

                                event_log_t entry;
                                memcpy(entry.cat_id, cat_id, sizeof(entry.cat_id));
                                entry.amount = prev_sec_sample_g - box_weight;
                                entry.type = (prev_sec_sample_g > box_weight) ? (poo_detected ? 2 : 1) : 3;
                                entry.cat_weight = weight_during_deposit - prev_sec_sample_g;
                                entry.samples_cnt = cnt;
                                entry.rawDate = samples_buffer;
                                entry.duration_sec = cnt / SPS;
                                xQueueSend(logger_queue, &entry,0);

                                box_weight = prev_sec_sample_g;
                                state = idle;
                            }
                    }
                    else
                        stable_sec_cnt = 0;
                }
                cnt++            
                break;
        }
    }
}

void sampler_init(void)
{
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, NUM_OF_CAT_IDS_TO_BUFF * SIZE_OF_CAT_ID_UART_RX, NUM_OF_UART_COMMANDS_TO_BUFF * SIZE_OF_UART_COMMAND_RX, 3, &tag_queue, 0));
    const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = 
    {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, RFID_TX, RFID_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

}