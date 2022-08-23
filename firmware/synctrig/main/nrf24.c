/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"

// nRF24L01Plus Table 19
#define R_REGISTER(a)       ((a)&0x1f)
#define W_REGISTER(a)       (((a)&0x1f)|0x20)
#define R_RX_PAYLOAD        0x61
#define W_TX_PAYLOAD        0xa0
#define FLUSH_TX            0xe1
#define FLUSH_RX            0xe2
#define REUSE_TX_PL         0xe3
#define R_RX_PL_WID         0x60
#define W_ACK_PAYLOAD(p)    (((p)&0x7)|0xa8)
#define W_TX_PAYLOAD_NOACK  0xb0

// Register map Table 27
#define REG_CONFIG 0
# define MASK_RX_DR         (1<<6)
# define MASK_TX_DS         (1<<5)
# define MASK_MAX_RT        (1<<4)
# define EN_CRC             (1<<3)
# define CRCO               (1<<2)
# define PWR_UP             (1<<1)
# define PRIM_RX            (1<<0)
#define REG_EN_AA 1
# define ENAA_P5            (1<<5)
# define ENAA_P4            (1<<4)
# define ENAA_P3            (1<<3)
# define ENAA_P2            (1<<2)
# define ENAA_P1            (1<<1)
# define ENAA_P0            (1<<0)
#define REG_EN_RXADDR 2
# define ERX_P5             (1<<5)
# define ERX_P4             (1<<4)
# define ERX_P3             (1<<3)
# define ERX_P2             (1<<2)
# define ERX_P1             (1<<1)
# define ERX_P0             (1<<0)
#define REG_SETUP_AW 3
# define AW_3BYTES          1
# define AW_4BYTES          2
# define AW_5BYTES          3
#define SETUP_RETR 4
# define ARD_ARC(d,c)       ((((d)&0xf)<<4)|((c)&0xf))
#define REG_RF_CH 5
#define REG_RF_SETUP 6
# define CONT_WAVE          (1<<7)
# define RF_DR_LOW          (1<<5)
# define PLL_LOCK           (1<<4)
# define RF_DR_HIGH         (1<<3)
# define RF_PWR(p)          (((p)&3)<<1)
#define REG_STATUS 7
# define RX_DR              (1<<6)
# define TX_DS              (1<<5)
# define MAX_RT             (1<<4)
# define RX_P_NO(r)         (((r)>>1)&7)
# define TX_FIFO_FULL       (1<<0)
#define REG_OBSERVE_TX 8
# define PLOS_CNT(r)        (((r)>>4)&0xf)
# define ARC_CNT(r)         ((r)&0xf)
#define REG_PRD 9
# define PRD                (1<<0)
#define REG_RX_ADDR_P0 0xa
#define REG_RX_ADDR_P1 0xb
#define REG_RX_ADDR_P2 0xc
#define REG_RX_ADDR_P3 0xd
#define REG_RX_ADDR_P4 0xe
#define REG_RX_ADDR_P5 0xf
#define REG_TX_ADDR 0x10
#define REG_RX_PW_P0 0x11
#define REG_RX_PW_P1 0x12
#define REG_RX_PW_P2 0x13
#define REG_RX_PW_P3 0x14
#define REG_RX_PW_P4 0x15
#define REG_RX_PW_P5 0x16
#define REG_FIFO_STATUS 0x17
# define TX_REUSE           (1<<6)
# define TX_FULL            (1<<5)
# define TX_EMPTY           (1<<4)
# define RX_FULL            (1<<1)
# define RX_EMPTY           (1<<0)
#define REG_DYNPD 0x1c
# define DPL_P5             (1<<5)
# define DPL_P4             (1<<4)
# define DPL_P3             (1<<3)
# define DPL_P2             (1<<2)
# define DPL_P1             (1<<1)
# define DPL_P0             (1<<0)
# define DPL_PALL           0x3f
#define REG_FEATURE 0x1d
# define EN_DPL             (1<<2)
# define EN_ACK_PAY         (1<<1)
# define EN_DYN_ACK         (1<<0)

// INT and CE pin
#define GPIO_NRF24_INT	GPIO_NUM_33
#define GPIO_NRF24_CE	GPIO_NUM_21

extern spi_device_handle_t spi_nrf24;

static esp_err_t nrf24_readn(uint8_t reg, uint8_t *buf, size_t len)
{
    esp_err_t ret;
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(spi_transaction_t));
    trans.cmd = reg;
    trans.length = 8*len;
    trans.flags = SPI_TRANS_USE_RXDATA;
    //printf("do transfer\n");
    //Queue all transactions.
    ret = spi_device_transmit(spi_nrf24, &trans);
    if (ret != ESP_OK) {
        return ret;
    }
    memcpy(buf, &trans.rx_data[0], len);
    return ret;
}

static esp_err_t nrf24_writen(uint8_t reg, uint8_t *buf, size_t len)
{
    esp_err_t ret;
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(spi_transaction_t));
    trans.cmd = reg;
    trans.length = 8*len;
    trans.tx_buffer = buf;
    trans.flags = 0;//SPI_TRANS_USE_RXDATA;
    //printf("do transfer buf %p len %d\n", buf, len);
    //Queue all transactions.
    ret = spi_device_transmit(spi_nrf24, &trans);
    if (ret != ESP_OK) {
        return ret;
    }
    return ret;
}

extern QueueHandle_t nrf24_evt_queue;

#define BUTTON_PIN	GPIO_NUM_33
void nrf24_task(void* arg)
{
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);

    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(GPIO_NRF24_CE, 0);

    // 8-bit CRC
    uint8_t val;
    val = EN_CRC; //(EN_CRC|CRCO);
    nrf24_writen(W_REGISTER(REG_CONFIG), &val, 1);
    // 2MBPS data rate
    nrf24_readn(R_REGISTER(REG_RF_SETUP), &val, 1);
    val &= ~(RF_DR_LOW|RF_DR_HIGH);
    val |= RF_DR_HIGH;
    nrf24_writen(W_REGISTER(REG_RF_SETUP), &val, 1);
    // 5 byte address
    val = AW_5BYTES;
    nrf24_writen(W_REGISTER(REG_SETUP_AW), &val, 1);
    // Reset auto ack
    val = 0; //0x3f;
    nrf24_writen(W_REGISTER(REG_EN_AA), &val, 1);
    // Reset feature/dynpd
    val = EN_DPL|EN_DYN_ACK;
    nrf24_writen(W_REGISTER(REG_FEATURE), &val, 1);
    val = DPL_PALL;
    nrf24_writen(W_REGISTER(REG_DYNPD), &val, 1);
    // Reset status
    val = RX_DR|TX_DS|MAX_RT;
    nrf24_writen(W_REGISTER(REG_STATUS), &val, 1);
    // Set default channel 76
    val = 90;//76;
    nrf24_writen(W_REGISTER(REG_RF_CH), &val, 1);
    nrf24_writen(FLUSH_RX, NULL, 0);
    nrf24_writen(FLUSH_TX, NULL, 0);
    // Power up
    nrf24_readn(R_REGISTER(REG_CONFIG), &val, 1);
    if (!(val & PWR_UP)) {
        val |= PWR_UP;
        printf("set config %02x\n", val);
        nrf24_writen(W_REGISTER(REG_CONFIG), &val, 1);
        vTaskDelay(2/portTICK_PERIOD_MS);
    }
    // Default PTX
    nrf24_readn(R_REGISTER(REG_CONFIG), &val, 1);
    val &= ~PRIM_RX;
    nrf24_writen(W_REGISTER(REG_CONFIG), &val, 1);

    // Set address of P0
    //uint8_t p0_addr[5] = { '2', 'N', 'o', 'd', 'e' };
    uint8_t p0_addr[5] = { 'F', 'G', 'H', 'I', 'J' };
    nrf24_writen(W_REGISTER(REG_RX_ADDR_P0), p0_addr, 5);
    // Set  address
    //uint8_t addr[5] = { '1', 'N', 'o', 'd', 'e' };
    uint8_t addr[5] = { 'F', 'G', 'H', 'I', 'J' };
    nrf24_writen(W_REGISTER(REG_TX_ADDR), addr, 5);

    int button = gpio_get_level(BUTTON_PIN);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        TickType_t lap = xTaskGetTickCount() - xLastWakeTime;
        if (100/portTICK_PERIOD_MS > lap) {
            vTaskDelay(100/portTICK_PERIOD_MS - lap);

        }
        xLastWakeTime = xTaskGetTickCount();

        if (button == 0 || gpio_get_level(BUTTON_PIN) != 0) {
            button = gpio_get_level(BUTTON_PIN);
            continue;
        }
        button = 0;

        uint8_t buf[32] = "hi!!";
        nrf24_writen(W_TX_PAYLOAD_NOACK, buf, 4);
        // This makes ~50us CE pulse
        gpio_set_level(GPIO_NRF24_CE, 1);
        nrf24_readn(R_REGISTER(REG_STATUS), &val, 1);
        gpio_set_level(GPIO_NRF24_CE, 0);

        uint32_t num;
        while (xQueueReceive(nrf24_evt_queue, &num, portMAX_DELAY) != pdTRUE)
            ;
        //vTaskDelay(1/portTICK_PERIOD_MS);
        nrf24_readn(R_REGISTER(REG_STATUS), &val, 1);
        printf("status %02x\n", val);
        val |= RX_DR|TX_DS|MAX_RT;
        nrf24_writen(W_REGISTER(REG_STATUS), &val, 1);
    }
}
