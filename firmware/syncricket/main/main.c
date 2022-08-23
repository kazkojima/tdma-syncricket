/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "hal/i2s_ll.h"

static const char *TAG = "SynCricket";

// Pin setting

#define I2S_BCK_PIN			I2S_PIN_NO_CHANGE
#define I2S_LRCK_PIN		GPIO_NUM_17
#define I2S_DATA_PIN		GPIO_NUM_16
#define I2S_DATA_IN_PIN		I2S_PIN_NO_CHANGE
#define I2S_NUMBER			I2S_NUM_0

#define PIN_NUM_MISO		GPIO_NUM_19
#define PIN_NUM_MOSI 		GPIO_NUM_23
#define PIN_NUM_CLK			GPIO_NUM_18
#define PIN_NUM_CS			GPIO_NUM_5

#define GPIO_NRF24_INT		GPIO_NUM_22
#define GPIO_NRF24_CE		GPIO_NUM_21

#define BUTTON_PIN			GPIO_NUM_39

// WiFi
static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

int s_retry_num;

// Event handler
void event_handler(void* arg, esp_event_base_t event_base,
                   int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < CONFIG_WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta()
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = { 0 };
    strlcpy((char *) wifi_config.sta.ssid, CONFIG_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *) wifi_config.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    /* Setting a password implies station will connect to all security modes including WEP/WPA.
     * However these modes are deprecated and not advisable to be used. Incase your Access point
     * doesn't support WPA2, these mode can be enabled by commenting below line */
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

#define TCP_SERVER CONFIG_TCP_SERVER_ADDRESS
#define TCP_PORT CONFIG_TCP_PORT

// NRF24 SPI

spi_device_handle_t spi_nrf24;

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

    // Simple level input
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1LL<<BUTTON_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_set_level(GPIO_NRF24_CE, 0);
}

#define NRF24_EVT_QSIZE 10
#define ESP_INTR_FLAG_DEFAULT 0

QueueHandle_t nrf24_evt_queue = NULL;

static volatile int sync_trig;

static void IRAM_ATTR nrf24_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    sync_trig = 1;
    xQueueSendFromISR(nrf24_evt_queue, &gpio_num, NULL);
}

#define DEFAULT_RATE 44100
static int32_t sampling_rate = DEFAULT_RATE;

static void pdmout_init(void)
{
    esp_err_t err = ESP_OK;

    //i2s_driver_uninstall(I2S_NUMBER);
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER),
        .sample_rate = DEFAULT_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_desc_num = 8,
        .dma_frame_num = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0,
    };
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_PDM);
    err += i2s_driver_install(I2S_NUMBER, &i2s_config, 0, NULL);
    i2s_pin_config_t tx_pin_config;

    tx_pin_config.bck_io_num = I2S_BCK_PIN;
    tx_pin_config.ws_io_num = I2S_LRCK_PIN;
    tx_pin_config.data_out_num = I2S_DATA_PIN;
    tx_pin_config.data_in_num = I2S_DATA_IN_PIN;

    err += i2s_set_pin(I2S_NUMBER, &tx_pin_config);
    err += i2s_set_clk(I2S_NUMBER, sampling_rate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    i2s_pdm_tx_upsample_cfg_t tx_upsample_config;
    tx_upsample_config.sample_rate = sampling_rate;
    tx_upsample_config.fp = sampling_rate / 100;
    tx_upsample_config.fs = sampling_rate / 100;
    err += i2s_set_pdm_tx_up_sample(I2S_NUMBER, &tx_upsample_config);
}

const uint8_t *sounddata;
ssize_t sizeof_sounddata = sizeof(short)*44100*10;

static void pdmout_task(void *arg)
{
    pdmout_init();

    i2s_zero_dma_buffer(I2S_NUMBER);

#define USE_I2S_DMA 1
#ifndef USE_I2S_DMA
    i2s_dev_t *hw = &I2S0;
    uint32_t fifo = 0x6000f000;
    i2s_ll_tx_disable_intr(hw); // Stop DMA tx intr
    i2s_ll_tx_stop_link(hw); // Stop DMA tx
    i2s_ll_rx_disable_intr(hw); // Stop DMA rx intr
    i2s_ll_rx_stop_link(hw); // Stop DMA rx
    //i2s_ll_tx_stop(hw); // Stop tx
    i2s_ll_rx_stop(hw); // Stop rx
    hw->fifo_conf.dscr_en = 0;
    hw->fifo_conf.tx_data_num = 32;

    uint32_t td = 2*1000000/sampling_rate - 1;

    while(1) {
        while(!sync_trig) {
            cpu_ll_waiti();// asm volatile ("waiti 0\n");
            asm volatile ("nop\n");
        }
        sync_trig = 0;

        const uint8_t *s = sounddata;
        ssize_t n = sizeof_sounddata;
        //REG_WRITE(fifo, 0); // Dummy
        //i2s_ll_tx_start(hw); // Start tx
        while(n > 0) {
            uint32_t dat = (s[1]<<24)|(s[0]<<16)|(s[3]<<8)|s[2];
            s += 4;
            n -= 4;
#if 0
            while(hw->int_st.tx_put_data) {
                usleep(300);
                hw->int_clr.put_data = 1;
            }
            usleep(44);
            REG_WRITE(fifo, dat);
#else
            if (!hw->int_raw.tx_wfull) {
                REG_WRITE(fifo, dat);
                usleep(td);
            } else {
                hw->int_clr.put_data = 1;
                while(!hw->int_raw.tx_put_data)
                    usleep(10);
                hw->int_clr.tx_wfull = 1;
            }
#endif
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
#else
    while(1) {
        while(!sync_trig) {
            cpu_ll_waiti();// asm volatile ("waiti 0\n");
            asm volatile ("nop\n");
        }
        sync_trig = 0;
        size_t nbytes;
        i2s_write(I2S_NUMBER, (const char *)sounddata, sizeof_sounddata,
                  &nbytes, portMAX_DELAY);
    }
#endif
}

// Storage
const esp_partition_t *partition;

// WAV

struct wavheader {
    char riff[4];
    int32_t chunk_size;
    char format[4];
    char fmt_id[4];
    int32_t fmt_chunk_size;
    int16_t audio_format;
    int16_t nchannel;
    int32_t sampling_rate;
    int32_t byte_per_sec;
    int16_t block_size;
    int16_t nbits;
    char data_id[4];
    int32_t data_length;
};

static bool parse_wav(struct wavheader *hp)
{
    if (0 != memcmp(hp->riff, "RIFF", 4))
        return false;
    if (0 != memcmp(hp->format, "WAVE", 4))
        return false;
    if (0 != memcmp(hp->fmt_id, "fmt ", 4))
        return false;
    if (hp->fmt_chunk_size != 16)
        return false;
    if (hp->audio_format != 1
        || hp->nchannel != 1
        || hp->block_size != 2
        || hp->nbits != 16)
        return false;
    if (0 != memcmp(hp->data_id, "data ", 4))
        return false;
    if (hp->sampling_rate < 8000 || hp->sampling_rate > 192000)
        return false;
    return true;
}

static inline ssize_t align4k(ssize_t n)
{
    return (n + (4096-1)) & ~(4096-1);
}

// Update sounddata via wifi
void update_task(void *arg)
{
    wifi_init_sta();

    struct sockaddr_in saddr;
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = inet_addr(CONFIG_TCP_SERVER_ADDRESS);
    saddr.sin_port = htons(CONFIG_TCP_PORT);
    int sock;
    int retry_connect = 5;
    while (true) {
        sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock < 0) {
            ESP_LOGE(TAG, "TCP socket can't be opened");
            vTaskDelete(NULL);
        }
        if (0 == connect (sock, (struct sockaddr *)&saddr, sizeof(saddr)))
            break;
        close(sock);
        if (--retry_connect <= 0) {
            ESP_LOGE(TAG, "TCP socket can't be opened");
            vTaskDelete(NULL);
        }
        ESP_LOGI(TAG, "Retrying to connect");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "Successfully connected");

    if (ESP_OK != esp_partition_erase_range(partition, 0,
                                            align4k(sizeof_sounddata))) {
        ESP_LOGE(TAG, "Failed to erase partition");
        vTaskDelete(NULL);
    }

#define BUF_SIZE 1024
    uint8_t buf[BUF_SIZE];
    ssize_t index = 0;
    ssize_t n;

    n = recv(sock, buf, sizeof(struct wavheader), 0);
    if (n < 0) {
        ESP_LOGE(TAG, "Failed to recv TCP socket errno %d", errno);
        vTaskDelete(NULL);
    } else if (n < sizeof(struct wavheader)) {
        ESP_LOGE(TAG, "Only get %d bytes", n);
        vTaskDelete(NULL);
    }

    // Parse WAV header if there
    struct wavheader *hp = (struct wavheader *)buf;
    if (parse_wav(hp)) {
        bool update_nvs = false;
        nvs_handle_t nvs_handle;
        if (hp->sampling_rate != sampling_rate
            || hp->data_length != sizeof_sounddata) {
            update_nvs = true;
            ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));
        }
        if (hp->sampling_rate != sampling_rate) {
            nvs_set_i32(nvs_handle, "sampling_rate", hp->sampling_rate);
        }
        if (hp->data_length != sizeof_sounddata) {
            nvs_set_i32(nvs_handle, "data_length", hp->data_length);
            sizeof_sounddata = hp->data_length;
        }

        if (update_nvs) {
            nvs_commit(nvs_handle);
            nvs_close(nvs_handle);
        }
        ESP_LOGI(TAG, "WAV rate=%d length=%d",
                 hp->sampling_rate, hp->data_length);
    } else {
        // Treat it as the raw S16_LE
        ESP_LOGI(TAG, "RAW rate=%d length=%d",
                 sampling_rate, sizeof_sounddata);
        // Read more bytes to get first BUF_SIZE bytes all
        n = recv(sock, buf+sizeof(struct wavheader),
                 BUF_SIZE-sizeof(struct wavheader), 0);
        if (n < 0) {
            ESP_LOGE(TAG, "Failed to recv TCP socket errno %d", errno);
            vTaskDelete(NULL);
        }
        if (n < BUF_SIZE-sizeof(struct wavheader)) {
            ESP_LOGE(TAG, "Too short sound data");
            vTaskDelete(NULL);
        }
        if (ESP_OK != esp_partition_write(partition, index, buf, BUF_SIZE)) {
            ESP_LOGE(TAG, "Failed to write partition");
            vTaskDelete(NULL);
         }
        index += BUF_SIZE;
    }

    while (true) {
        n = recv(sock, buf, BUF_SIZE, 0);
        if (n < 0) {
            ESP_LOGE(TAG, "Failed to recv TCP socket errno %d", errno);
            vTaskDelete(NULL);
        }
        if (index + n >= sizeof_sounddata)
            n = sizeof_sounddata - index;
        if (n == 0) {
            break;
        }
        if (ESP_OK != esp_partition_write(partition, index, buf, n)) {
            ESP_LOGE(TAG, "Failed to write partition");
            vTaskDelete(NULL);
         }
        index += n;
        if (index >= sizeof_sounddata) {
            break;
        }
    }

    ESP_LOGI(TAG, "sound data updated");
    vTaskDelete(NULL);
}

extern void nrf24_task(void *arg);

void app_main(void)
{
    esp_err_t err;

    //Initialize NVS
    nvs_flash_init();

    spi_init();
    xgpio_init();

    nvs_handle_t nvs_handle;
    //Get some constants from NVS. If not in NVS, use default
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));

    int32_t rate, size;
    err = nvs_get_i32(nvs_handle, "sampling_rate", &rate);
    if (err == ESP_OK && rate >= 8000 && rate <= 192000) {
        sampling_rate = rate;
    } else {
        ESP_LOGI(TAG, "Use default rate %d", DEFAULT_RATE);
        sampling_rate = DEFAULT_RATE;
    }
    err = nvs_get_i32(nvs_handle, "data_length", &size);
    if (err == ESP_OK && size >= 0 && size < 2*1024*1024) {
        sizeof_sounddata = size;
    } else {
        ESP_LOGI(TAG, "Use default size %d", sizeof(short)*DEFAULT_RATE*10);
        sizeof_sounddata = sizeof(short)*DEFAULT_RATE*10;
    }

    nvs_close(nvs_handle);

    // Find the partition map in the partition table
    partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    assert(partition != NULL);
    ESP_LOGI(TAG, "Found storage partition of size %d", partition->size);

    // Update operation when button is held
    int hold = 0;
    while (gpio_get_level(BUTTON_PIN) == 0) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if (++hold > 20) {
            // Invoke update sound data task and do idle loop
            xTaskCreate(update_task, "update", 8196, NULL, 2, NULL);
            while (1) {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    }

    // Normal operation
    spi_flash_mmap_handle_t map_handle;
    // Map the partition to sound memory
    ESP_ERROR_CHECK(esp_partition_mmap(partition, 0, partition->size, SPI_FLASH_MMAP_DATA, (const void**)&sounddata, &map_handle));
    ESP_LOGI(TAG, "Mapped partition to data memory address %p", sounddata);

    // Sound property
    ESP_LOGI(TAG, "PDM rate=%d data_length=%d", sampling_rate, sizeof_sounddata);

    // NRF24 event queue
    nrf24_evt_queue = xQueueCreate(NRF24_EVT_QSIZE, sizeof(uint32_t));
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_NRF24_INT, nrf24_isr_handler,
                         (void *)GPIO_NRF24_INT);


    xTaskCreate(nrf24_task, "nrf24", 2048, NULL, 5, NULL);
    xTaskCreatePinnedToCore(pdmout_task, "pdmout", 8192, NULL, 1, NULL, 1);
                           
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true) {
        gpio_set_level(GPIO_NUM_2, level);
        level = !level;
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

