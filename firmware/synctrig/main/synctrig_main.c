/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"

esp_err_t event_handler(void *ctx, esp_event_base_t event_base,
                        int32_t event_id, void* event_data)
{
    switch(event_id) {
    default:
        break;
    }
    return ESP_OK;
}

spi_device_handle_t spi_nrf24;

#define PIN_NUM_MISO 36
#define PIN_NUM_MOSI 19
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   25

static void spi_init(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=0
    };
    spi_device_interface_config_t devcfg={
        .command_bits=8,
        .address_bits=0,
        .dummy_bits=0,
        .clock_speed_hz=8000000,                //Clock out at 8MHz
        .duty_cycle_pos=128,
        .mode=0,                                //SPI mode 0
        .cs_ena_posttrans=1,
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=1,                          //queue size
        .flags=0,
    };

    //Initialize the SPI bus
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 0);
    assert(ret==ESP_OK);
    //Attach the slave devices to the SPI bus
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi_nrf24);
    assert(ret==ESP_OK);
}

#define GPIO_NRF24_INT	GPIO_NUM_22
#define GPIO_NRF24_CE	GPIO_NUM_21

static void xgpio_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1LL<<GPIO_NRF24_CE);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1LL<<GPIO_NRF24_INT);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(GPIO_NRF24_CE, 0);
}

#define NRF24_EVT_QSIZE 10
#define ESP_INTR_FLAG_DEFAULT 0

QueueHandle_t nrf24_evt_queue = NULL;

static void IRAM_ATTR nrf24_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(nrf24_evt_queue, &gpio_num, NULL);
}

extern void nrf24_task(void *arg);

void app_main(void)
{
    spi_init();
    xgpio_init();

    // NRF24 event queue
    nrf24_evt_queue = xQueueCreate(NRF24_EVT_QSIZE, sizeof(uint32_t));
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_NRF24_INT, nrf24_isr_handler,
                         (void *)GPIO_NRF24_INT);


    xTaskCreate(nrf24_task, "nrf24_task", 2048, NULL, 5, NULL);

    //gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true) {
        //gpio_set_level(GPIO_NUM_2, level);
        level = !level;
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

