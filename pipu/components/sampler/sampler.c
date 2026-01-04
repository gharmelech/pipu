#include <stdio.h>
#include "sampler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "logger.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#define TAG "Sampler"

#define CONVERTION_FACTOR 70
#define SPS 80
#define CAT_THRESH_G 1000
#define DEFAULT_BOX_WEIGHT_G 8000
#define NO_CHANGE_THRESH_G 10
#define POO_DETECTION_THRESH_G 2
#define MAX_DEPOSIT_LEN_S 300
#define MAX_SAMPLES_BETWEEN_TRANSIENT_PARTS (SPS / 10) // transient should be less than 0.1 sec
#define SETTLING_TIME_S 5
#define NUM_OF_CAT_IDS_TO_BUFF 3
#define BYTES_IN_CAT_ID_UART_RX 8
#define BYTES_IN_COUNTRY_CODE 2
#define BYTES_IN_ID 5
#define NUM_OF_UART_COMMANDS_TO_BUFF 3
#define BYTES_IN_UART_COMMAND 1
#define CAT_ID_STRING_LENGTH 15
#define START_SCAN 0xAA
#define STOP_SCAN 0xBB
#define DEFAULT_CAT_ID "000000000000000"
#define SAMPLE_BITS 24

static spi_device_handle_t scale_spi_handle;
static QueueHandle_t tag_queue;
static QueueHandle_t weight_queue;
static TaskHandle_t take_weight_task_handle = NULL;
static uint16_t offset = 0;
static state_t state = idle;
static volatile bool lid_off = false;
static volatile bool chip_scanned = false;
static char cat_id[CAT_ID_STRING_LENGTH + 1]; //15 chars + string termination

static void chip_scanner_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t data[BYTES_IN_CAT_ID_UART_RX];
    while (1)
    {
        if (xQueueReceive(tag_queue, &event, portMAX_DELAY))
        {
            // first, stop scanner
            uint8_t stop_cmd = STOP_SCAN;
            uart_write_bytes(RFID_UART, (const char*)&stop_cmd, 1);
            switch (event.type) // parse reading
            {
                case UART_DATA:
                    int len = uart_read_bytes(RFID_UART, data, event.size, portMAX_DELAY);
                    if (len == BYTES_IN_CAT_ID_UART_RX)
                    {
                        uint8_t chksum = 0;
                        for (int i = 0; i < BYTES_IN_CAT_ID_UART_RX - 1; i++)
                            chksum ^= data[i];
                        if (chksum == data[BYTES_IN_CAT_ID_UART_RX - 1]) // all good
                        {
                            uint16_t contry_code= 0;
                            for (int i = 0; i < BYTES_IN_COUNTRY_CODE; i++)
                                contry_code = (contry_code << 8) | data[BYTES_IN_COUNTRY_CODE - 1 - i];
                            uint64_t id= 0;
                            for (int i = 0; i < BYTES_IN_ID; i++)
                                id = (id << 8) | data[BYTES_IN_CAT_ID_UART_RX - 2 - i];
                            id %= 1000000000000ULL;
                            contry_code %=1000;
                            snprintf(cat_id, sizeof(cat_id), "%03u%012llu", contry_code, id);
                            chip_scanned = true;
                        } 
                    }
                    else if (len == BYTES_IN_UART_COMMAND)
                        ESP_LOGI(TAG, "UART commend acknowledged");
                    else // error
                        ESP_LOGI(TAG, "UART error");
                    break;
                default:
                    ESP_LOGI(TAG, "Unexpected UART event");
                    break;
            }

            // flush to avoid more triggers
            uart_flush(RFID_UART);
            xQueueReset(tag_queue);
        }
    }
}

uint8_t scale_write_byte (uint8_t byte)
{
    portMUX_TYPE spiMux = portMUX_INITIALIZER_UNLOCKED;
    spi_transaction_t t =
    {
        .flags      = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .tx_data[0] = byte,
        .length     = 16,
        .rxlength   = 8,
    };
    taskENTER_CRITICAL(&spiMux);
    spi_device_polling_transmit(scale_spi_handle, &t);
    taskEXIT_CRITICAL(&spiMux);
    return(t.rx_data[0]);
}

void take_weight_sample()
{
    static uint8_t scale_cells[4] = {SCALE_CELL0, SCALE_CELL1, SCALE_CELL2, SCALE_CELL3};
    int32_t sample = 0;
    int32_t cell_sample = 0;

    ESP_LOGI(TAG, "starting ADC sampler task.");
    vTaskDelay(pdMS_TO_TICKS(3000));
    xTaskNotifyStateClear(take_weight_task_handle);
    ulTaskNotifyValueClear(take_weight_task_handle, 0xFFFFFFFF);

    // send reset
    gpio_set_level((gpio_num_t)SCALE_RST, 0);
    vTaskDelay(10);
    gpio_set_level((gpio_num_t)SCALE_RST, 1);
    vTaskDelay(2000);

    // wait for DRDY (indicating out of reset)
    while (gpio_get_level((gpio_num_t)SCALE_DRDY))
    {
        esp_rom_delay_us(10);
    } 

    // // tie cs low
    gpio_set_level((gpio_num_t)SCALE_CS,0);

    //set data rate
    esp_rom_delay_us(20);
    scale_write_byte(SCALE_CMD_WREG | SCALE_DRATE_REG);
    scale_write_byte(0x00);
    scale_write_byte(SCALE_2_5SPS);
    
    esp_rom_delay_us(20);
    
    //set PGA
    esp_rom_delay_us(20);
    scale_write_byte(SCALE_CMD_WREG | SCALE_ADCON_REG);
    scale_write_byte(0x00);
    scale_write_byte(SCALE_GAIN64);
    
    esp_rom_delay_us(20);
    
    //set cell3
    esp_rom_delay_us(20);
    scale_write_byte(SCALE_CMD_WREG | SCALE_MUX_REG);
    scale_write_byte(0x00);
    scale_write_byte(scale_cells[3]);
    
    esp_rom_delay_us(20);
    
    
    //verify register values
    for (int i = 0; i < 4; i++)
    {
        esp_rom_delay_us(10);
        scale_write_byte(SCALE_CMD_RREG);
        esp_rom_delay_us(10);
        scale_write_byte(0x00);
        esp_rom_delay_us(10);
        sample = scale_write_byte(0xFF);
        ESP_LOGI(TAG, "Register %d readback: 0x%02X", i, sample);
    }
    
    esp_rom_delay_us(20);
    
    //selfcal
    esp_rom_delay_us(20);
    scale_write_byte(SCALE_CMD_SELFCALL);
    
    esp_rom_delay_us(20);
    while (gpio_get_level((gpio_num_t)SCALE_DRDY))
    {
        // ESP_LOGI(TAG, "Waiting for self cal");
        esp_rom_delay_us(10);
    } 
    
    //sync
    esp_rom_delay_us(20);
    scale_write_byte(SCALE_CMD_SYNC);
    
    esp_rom_delay_us(20);
    
    //wakeup
    esp_rom_delay_us(20);
    scale_write_byte(SCALE_CMD_WAKEUP);
    
    esp_rom_delay_us(20);
    
    //wait for data
    while (gpio_get_level((gpio_num_t)SCALE_DRDY))
    {
        // ESP_LOGI(TAG, "Waiting for data");
        esp_rom_delay_us(10);
    }
    
    //loop read data
    while(1)
    {
        sample = 0;
        
        // send rdata

        esp_rom_delay_us(20);
        scale_write_byte(SCALE_CMD_RDATA);
        esp_rom_delay_us(20);
        
        for (int i = 2; i >= 0; i--)
        {
            sample =  (sample << 8) | scale_write_byte(SCALE_CMD_RDATA);
            ESP_LOGI(TAG, "Byte readback: %X", sample);
            esp_rom_delay_us(10);
        }

        
        esp_rom_delay_us(20);
        if (sample & 0x00800000)
            sample = sample | 0xFF000000;
        ESP_LOGI(TAG, "Sample readback: %d, factored: %f", sample, (float)sample / 31.42);

        gpio_intr_enable((gpio_num_t)SCALE_DRDY);
        taskYIELD();
        ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY);
    }
}

void sampler_fsm(void *pvParameters)
{
    int32_t sample_raw = 0;
    uint16_t sample_g = 0;
    uint16_t prev_sample_g = 0;
    uint32_t one_sec_sample_g = 0;
    uint16_t prev_sec_sample_g = 0;
    uint16_t measurement_delta = 0;
    uint16_t min_g = 0;
    uint16_t box_weight = DEFAULT_BOX_WEIGHT_G;
    uint16_t weight_during_deposit = 0;
    uint16_t cnt = 0;
    uint16_t first_half_poo_transient = 0;
    bool cat_inside = false;
    bool poo_detected = false;
    uint8_t stable_sec_cnt = 0;
    int32_t *samples_buffer = NULL;

    vTaskDelay(pdMS_TO_TICKS(8000));

    while (1)
    {
        xQueueReceive(weight_queue, &sample_raw, portMAX_DELAY); // wait for raw sample
        sample_g = (sample_raw / CONVERTION_FACTOR) - offset;
        ESP_LOGI(TAG, "weight sample raw: %d, factorized: %d grams.", sample_raw, sample_g);
        switch (state)
        {
            case idle:
                if ((sample_g > (box_weight + CAT_THRESH_G)) || (chip_scanned)) // cat detacted
                {
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
                else if (lid_off)
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
                cnt++;
                one_sec_sample_g = one_sec_sample_g + sample_g;
                if (sample_g < min_g)
                    min_g = sample_g;
                if ((cnt / SPS > 0) && (cnt % SPS == 0)) // completed one second of samples
                {
                    one_sec_sample_g = one_sec_sample_g / SPS;
                    measurement_delta = abs((int32_t)prev_sec_sample_g - (int32_t)one_sec_sample_g);
                    prev_sec_sample_g = one_sec_sample_g;
                    one_sec_sample_g = 0;
                    lid_off = gpio_get_level((gpio_num_t)LID_DETECT);
                    if (!lid_off) //cleaning done
                    {
                        if (measurement_delta < NO_CHANGE_THRESH_G)
                        {
                            stable_sec_cnt++;
                            if (stable_sec_cnt >= SETTLING_TIME_S)
                            {
                                offset = offset + min_g;
                                box_weight = prev_sec_sample_g;
                                event_log_t entry;
                                sprintf(entry.cat_id, "cleaning");
                                entry.amount = 0;
                                entry.type = 0;
                                entry.cat_weight = box_weight;
                                entry.samples_cnt = 0;
                                entry.rawDate = NULL;
                                entry.duration_sec = 0;
                                xQueueSend(logger_queue, &entry,0);
                                gpio_intr_enable((gpio_num_t)LID_DETECT);
                                state = idle;
                            }
                        }
                        else
                            stable_sec_cnt = 0;
                    }
                }
                break;
            case deposit:
                if (samples_buffer != NULL)
                    samples_buffer[cnt++] = sample_raw;
                one_sec_sample_g = one_sec_sample_g + sample_g;
                if (cat_inside && !poo_detected) // poo detection
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
                    else if (((int32_t)prev_sample_g - (int32_t)sample_g >= 1) && ((int32_t)prev_sample_g - (int32_t)sample_g < NO_CHANGE_THRESH_G)) // very small drop in weight
                        first_half_poo_transient = MAX_SAMPLES_BETWEEN_TRANSIENT_PARTS;
                }
                prev_sample_g = sample_g;

                if ((cnt / SPS > 0) && (cnt % SPS == 0)) // completed one second of samples
                {
                    one_sec_sample_g = one_sec_sample_g / SPS;
                    measurement_delta = abs((int32_t)one_sec_sample_g - (int32_t)prev_sec_sample_g);
                    prev_sec_sample_g = one_sec_sample_g;
                    one_sec_sample_g = 0;
                    if (measurement_delta < NO_CHANGE_THRESH_G)
                    {
                        stable_sec_cnt++;
                        if (stable_sec_cnt >= SETTLING_TIME_S)
                        {
                            if (!cat_inside && ((int32_t)prev_sec_sample_g - (int32_t)box_weight > CAT_THRESH_G))
                            {
                                cat_inside = true;
                                weight_during_deposit = prev_sec_sample_g;
                            }
                            else if ((int32_t)weight_during_deposit - (int32_t)prev_sec_sample_g > CAT_THRESH_G)
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
                                snprintf(cat_id,sizeof(cat_id),DEFAULT_CAT_ID);
                                uint8_t start_cmd = START_SCAN; // re-enable scanner
                                uart_write_bytes(RFID_UART, (const char*)&start_cmd, 1);

                                state = idle;
                            }
                        }
                    }
                    else
                        stable_sec_cnt = 0;
                }
                break;
        }
    }
}

static void IRAM_ATTR lid_isr(void)
{
    gpio_intr_disable(LID_DETECT);
    lid_off = true;
}

static void IRAM_ATTR scale_isr(void)
{
    if (take_weight_task_handle != NULL)
    {
        gpio_intr_disable((gpio_num_t)SCALE_DRDY);
        BaseType_t takeWeightWoken = pdFALSE;
        vTaskGenericNotifyGiveFromISR(take_weight_task_handle, 0, &takeWeightWoken);
        portYIELD_FROM_ISR(takeWeightWoken);
    }
}

void sampler_init(void)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Init sampler");
    ESP_LOGI(TAG, "Configuring GPIO and UART");
    weight_queue = xQueueCreate(4, sizeof(int32_t));
    // ESP_ERROR_CHECK(uart_driver_install(RFID_UART, NUM_OF_CAT_IDS_TO_BUFF * BYTES_IN_CAT_ID_UART_RX, NUM_OF_UART_COMMANDS_TO_BUFF * BYTES_IN_UART_COMMAND, 3, &tag_queue, 0));
    ESP_ERROR_CHECK(uart_driver_install(RFID_UART, 256, 256, 20, &tag_queue, 0));
    uart_config_t uart_config = 
    {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config((uart_port_t)RFID_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(RFID_UART, RFID_TX, RFID_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    gpio_install_isr_service(0);
    
    // gpio for lid detection
    gpio_config_t current_gpio_config =
    {
       .pin_bit_mask = (1ULL << LID_DETECT),
       .mode = GPIO_MODE_INPUT,
       .pull_up_en = GPIO_PULLUP_ENABLE,
       .pull_down_en = GPIO_PULLDOWN_DISABLE,
       .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&current_gpio_config));
    gpio_intr_disable((gpio_num_t)LID_DETECT);
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)LID_DETECT, (gpio_isr_t)lid_isr, NULL));

    // gpio for scale DRDY pin
    current_gpio_config.pin_bit_mask = (1ULL << SCALE_DRDY);
    current_gpio_config.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&current_gpio_config));
    gpio_intr_disable((gpio_num_t)SCALE_DRDY);
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)SCALE_DRDY, (gpio_isr_t)scale_isr, NULL));

    current_gpio_config.pin_bit_mask = (1ULL << SCALE_RST);
    current_gpio_config.mode = GPIO_MODE_OUTPUT;
    current_gpio_config.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&current_gpio_config));

    current_gpio_config.pin_bit_mask = (1ULL << SCALE_CS);
    current_gpio_config.mode = GPIO_MODE_OUTPUT;
    current_gpio_config.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&current_gpio_config));
    gpio_set_level((gpio_num_t)SCALE_CS,1);


    // Set up scale SPI
    spi_bus_config_t scale_spi_bus_conf = {
        .miso_io_num = SCALE_MISO,
        .mosi_io_num = SCALE_MOSI,
        .sclk_io_num = SCALE_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ret = spi_bus_initialize(SCALE_HOST, &scale_spi_bus_conf, (spi_dma_chan_t)SPI_DMA_CH_AUTO);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "SPI bus init succesfull");
    }
    else
    {
        ESP_LOGI(TAG, "SPI bus init FAILED");
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
    
    spi_device_interface_config_t scale_spi_dev_conf = {
        .clock_speed_hz = 500000, //500Khz 1Mhz
        .mode = 1,
        // .input_delay_ns = 50,
        .spics_io_num = -1,
        .queue_size = 1,
        .command_bits = 0,
        // .flags = SPI_DEVICE_HALFDUPLEX,
    };
    ret = spi_bus_add_device(SCALE_HOST, &scale_spi_dev_conf, &scale_spi_handle);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "SPI device init succesfull");
    }
    else
    {
        ESP_LOGI(TAG, "SPI device init FAILED");
        vTaskDelay(pdMS_TO_TICKS(30000));
    }

    ESP_LOGI(TAG, "Starting task 1/3: chip scanner task");
    xTaskCreate(chip_scanner_task, "chip_scanner", 2048, NULL, 6, NULL);
    ESP_LOGI(TAG, "Starting task 2/3: ADC sampler");
    xTaskCreate(take_weight_sample, "take_weight", 4096, NULL, 9, &take_weight_task_handle);
    ESP_LOGI(TAG, "Starting task 3/3: sampler FSM");
    xTaskCreate(sampler_fsm, "sampler_fsm", 2048, NULL, 6, NULL);

    ESP_LOGI(TAG, "Init complete");
}
