/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 * 
 *  SLAVE - ACCEPTOR
 * 
 *  v00      - simplification of the code and removal of unnecessary parts of the code
 *  v00-01   - creation of a lower priority application task for printing received BT data outside of the callback function 
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"                // Added for blinking BT_LED
#include "freertos/queue.h"                 // Added to create a lower priority application task for printing BT data outside of the callback event
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "time.h"
#include "sys/time.h"

#include "driver/gpio.h"                                        

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define BT_DEVICE_NAME "ESP_SPP_ACCEPTOR"                  // Name for pairing, the MASTER will attempt to connect to a device with this name

#define LED_BT_PIN 2                                                  // GPIO2  LED BT
#define LED_DATA_PIN 27                                               // GPIO27 LED DATA

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const bool esp_spp_enable_l2cap_ertm = true;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

#define SPP_DATA_LEN 80                             // Length of transmitted data NOT USED NOW
static uint8_t spp_data[SPP_DATA_LEN];              
static uint8_t *s_p_data = NULL;                    // data pointer of spp_data

static uint32_t spp_handle = 0;                     // Global variable for storing SPP handle  NOT USED NOW

// Variables for handling BT_LED
static TimerHandle_t blink_timer = NULL;
static bool is_connected = false;

// Turn on DATA_LED on the specified pin (1) or turn off (0)
void toggle_data_led(bool state_data_led) {
    gpio_set_level(LED_DATA_PIN, state_data_led ? 1 : 0); 
}
// Function prototypes for BT_LED
void toggle_led_bt(bool state_bt_led);
void blink_timer_callback(TimerHandle_t xTimer);

// Definition of variables for creating a queue
static QueueHandle_t data_queue = NULL;
static void data_processing_task(void *pvParameter);
#define DATA_QUEUE_SIZE 100                                 // Queue size

// Function for blinking BT LED
void blink_timer_callback(TimerHandle_t xTimer) {
    if (!is_connected) {
        // Switching the state of the LED
        static bool led_state = false;
        toggle_led_bt(led_state);
        led_state = !led_state;
    }
}

// Modified function for turning on/off the BT LED
void toggle_led_bt(bool state_bt_led) {
    gpio_set_level(LED_BT_PIN, state_bt_led ? 1 : 0);
}

// LED initialization
static void init_led(void) {
    gpio_reset_pin(LED_DATA_PIN);
    gpio_set_direction(LED_DATA_PIN, GPIO_MODE_OUTPUT);
    ESP_LOGI(SPP_TAG, "Init DATA LED completed");
    gpio_reset_pin(LED_BT_PIN);
    gpio_set_direction(LED_BT_PIN, GPIO_MODE_OUTPUT);
    ESP_LOGI(SPP_TAG, "Init BT LED completed");
}

// Implementation of printing for data processing
static void data_processing_task(void *pvParameter) {
    ESP_LOGI(SPP_TAG, "Data processing task started");
    uint8_t data[SPP_DATA_LEN];
    while (1) {
        if (xQueueReceive(data_queue, &data, portMAX_DELAY)) {
            // Processing the received data here
            // For example, print the data in text form
            ESP_LOGI(SPP_TAG, "************************************Received data START**********************************************");
            char received_text[SPP_DATA_LEN + 1];
            memcpy(received_text, data, SPP_DATA_LEN);
            received_text[SPP_DATA_LEN] = '\0';
            ESP_LOGI(SPP_TAG, "Received data (TEXT): %s", received_text);
            ESP_LOGI(SPP_TAG, "************************************Received data END************************************************");
        }
    }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    char bda_str[18] = {0};
    
    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");      
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        if (param->open.status == ESP_SPP_SUCCESS) {
            spp_handle = param->open.handle; // Saving the handle
        }
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%"PRIu32" close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
               
        // When the connection is terminated
        is_connected = false;
        xTimerStart(blink_timer, 0); // Start blinking
                
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%"PRIu32" sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                     param->start.scn);
            esp_bt_dev_set_device_name(BT_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
        }
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        if (param->data_ind.len > 0) {
            // If some data is received, the LED will light up
            toggle_data_led(true);
            ESP_LOGI(SPP_TAG, "DATA LED ON");
            toggle_data_led(false);

            ESP_LOGI(SPP_TAG, "Data received, length: %d", param->data_ind.len);
            // Sending data to the queue
            if (xQueueSend(data_queue, param->data_ind.data, portMAX_DELAY) != pdPASS) {
                ESP_LOGE(SPP_TAG, "Failed to send data to queue");
            }
         } else {
        // If no data is received, the LED turns off
        toggle_data_led(false);
        ESP_LOGI(SPP_TAG, "DATA LED OFF");
        }
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_CONG_EVT:                     
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");

        // Displaying the address of the remote device
        ESP_LOGI(SPP_TAG, "Remote device address: %02x:%02x:%02x:%02x:%02x:%02x",
             param->srv_open.rem_bda[0], param->srv_open.rem_bda[1], param->srv_open.rem_bda[2],
             param->srv_open.rem_bda[3], param->srv_open.rem_bda[4], param->srv_open.rem_bda[5]);
        
        // When the connection is established
        is_connected = true;
        toggle_led_bt(true); // Permanently turn on the LED
        xTimerStop(blink_timer, 0); // Stop blinking   

        // Sending the text message "HELLO SLAVE HERE!"
        const char *message = "HELLO SLAVE HERE !";
        esp_spp_write(param->open.handle, strlen(message), (uint8_t *)message);
        break;
    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

void app_main(void)
{
    init_led();                             // Initialization of LED diodes

    // Initialization of the queue for data
    data_queue = xQueueCreate(DATA_QUEUE_SIZE, sizeof(uint8_t) * SPP_DATA_LEN);
    if (data_queue == NULL) {
        ESP_LOGE(SPP_TAG, "Failed to create queue");
        return;
    }

    // Creation of a task for data processing
    xTaskCreate(data_processing_task, "data_processing_task", 2048, NULL, 5, NULL);

    esp_err_t ret = nvs_flash_init();
    
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );                                                     // Verification if the previous function call was successful

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));            // Releasing memory allocated for Bluetooth Low Energy (BLE) 

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    
    esp_spp_cfg_t bt_spp_cfg = {
        .mode = esp_spp_mode,
        .enable_l2cap_ertm = esp_spp_enable_l2cap_ertm,
    };
    
    if ((ret = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;                              // ESP_BT_IO_CAP_NONE without the possibility of input and entering PIN (the original value was ESP_BT_IO_CAP_IO and required a PIN)
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    // Initialization of the timer for blinking
    blink_timer = xTimerCreate("BlinkTimer", pdMS_TO_TICKS(250), pdTRUE, (void *)0, blink_timer_callback);  // pdMS_TO_TICKS(250) sets the blinking speed in ms
    xTimerStart(blink_timer, 0); // Start blinking right at the beginning

}
