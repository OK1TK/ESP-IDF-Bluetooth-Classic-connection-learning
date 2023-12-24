/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 * 
 *  MASTER - INITIATOR
 * 
 *  v00 - simplification of the code and removal of unnecessary parts of the code
 *          The issue with reconnection still remains, likely caused by the implementation of the part for reading data from the slave
 * 
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

#define SPP_TAG "SPP_INITIATOR_DEMO"
#define EXAMPLE_DEVICE_NAME "ESP_SPP_INITIATOR"

#define LED_BT_PIN 2                                                  // GPIO2  LED BT
#define LED_DATA_PIN 27                                               // GPIO27 LED DATA

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const bool esp_spp_enable_l2cap_ertm = true;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_master = ESP_SPP_ROLE_MASTER;

esp_bd_addr_t peer_bd_addr = {0};                                               // Variable for storing the BDA (Bluetooth Device Address) of the remote device
static uint8_t peer_bdname_len;                                                 // Variable for storing the length of the remote device's name
static char peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];                         // Variable for storing the name of the remote device; array size = max. name length + 1 character
static const char remote_device_name[] = "ESP_SPP_ACCEPTOR";                    // Variable for storing the name of the device we are trying to connect to
static const esp_bt_inq_mode_t inq_mode = ESP_BT_INQ_MODE_GENERAL_INQUIRY;      // Variable determines the Bluetooth discovery mode. "General Inquiry" is the standard mode for discovering devices in range
static const uint8_t inq_len = 30;                                              // Variable for determining the length of the discovery in units of 1.28 sec. It specifies how long other devices will be searched for
static const uint8_t inq_num_rsps = 0;                                          // Variable determines the maximum number of responses to be processed during a single discovery cycle - 0 = no limit

#define SPP_DATA_LEN 80                                                         // Length of transmitted data NOT USED NOW
static uint8_t spp_data[SPP_DATA_LEN];
static uint8_t *s_p_data = NULL;                                                // data pointer of spp_data

static uint32_t spp_handle = 0;                                                 // Global variable for storing SPP handle  NOT USED NOW

// Variables for handling BT_LED
static TimerHandle_t blink_timer = NULL;
static bool is_connected = false;

// Variable for the reconnect timer
static TimerHandle_t reconnect_timer = NULL;

// Converting Bluetooth Device Address (BDA) into a readable format
static char *bda2str(uint8_t * bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}
// Function for extracting the name of a Bluetooth device from Extended Inquiry Response (EIR) data received during the discovery process
static bool get_name_from_eir(uint8_t *eir, char *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

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
#define DATA_QUEUE_SIZE 100                              // Velikost fronty

// Function for blinking BT LED
void blink_timer_callback(TimerHandle_t xTimer) {
    if (!is_connected) {
        // Přepínání stavu LED
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

// Function for attempting reconnection
void reconnect_timer_callback(TimerHandle_t xTimer) {
    // Attempting reconnection
    if (!is_connected)
    {
        esp_spp_start_discovery(peer_bd_addr);
    }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    uint8_t i = 0;
    char bda_str[18] = {0};

    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        if (param->disc_comp.status == ESP_SPP_SUCCESS) {                                               // Was the device search successful ?
            ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT scn_num:%d", param->disc_comp.scn_num);       // Logging the number of services found
            for (i = 0; i < param->disc_comp.scn_num; i++) {                                            // The loop goes through all the found services, logging their SCN and service name
                ESP_LOGI(SPP_TAG, "-- [%d] scn:%d service_name:%s", i, param->disc_comp.scn[i],
                         param->disc_comp.service_name[i]);
            }
            /* We only connect to the first found server on the remote SPP acceptor here */
            esp_spp_connect(sec_mask, role_master, param->disc_comp.scn[0], peer_bd_addr);              // Attempt to establish a connection with the first found server
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT status=%d", param->disc_comp.status);         // If the connection attempt fails, an error is logged
            // If the device is not connected, restart the discovery process
            esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
        }
        break;
    case ESP_SPP_OPEN_EVT:
        if (param->open.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT handle:%"PRIu32" rem_bda:[%s]", param->open.handle,
                     bda2str(param->open.rem_bda, bda_str, sizeof(bda_str)));
            
            // When the connection is established
            is_connected = true;
            toggle_led_bt(true); // Permanently turn on the LED
            xTimerStop(blink_timer, 0); // Stop blinking

            // Sending the text message "HELLO MASTER HERE!"
            const char *message = "HELLO MASTER HERE !";
            esp_spp_write(param->open.handle, strlen(message), (uint8_t *)message);


            } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_OPEN_EVT status:%d", param->open.status);
            }
            break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%"PRIu32" close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        
        // When the connection is terminated
        is_connected = false;
        xTimerStart(blink_timer, 0); // Start blinking
        xTimerStart(reconnect_timer, 0); // Start the timer for reconnection

        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        if (param->cl_init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT handle:%"PRIu32" sec_id:%d", param->cl_init.handle, param->cl_init.sec_id);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_CL_INIT_EVT status:%d", param->cl_init.status);
        }
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT");

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
        // If no data is received, the LED will turn of                       
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
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch(event){
    case ESP_BT_GAP_DISC_RES_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_RES_EVT");
        esp_log_buffer_hex(SPP_TAG, param->disc_res.bda, ESP_BD_ADDR_LEN);
        /* Find the target peer device name in the EIR data */
        for (int i = 0; i < param->disc_res.num_prop; i++){
            if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR
                && get_name_from_eir(param->disc_res.prop[i].val, peer_bdname, &peer_bdname_len)){
                esp_log_buffer_char(SPP_TAG, peer_bdname, peer_bdname_len);

                // Print the peer device name
                ESP_LOGI(SPP_TAG, "Peer device name: %.*s", peer_bdname_len, peer_bdname);

                if (strlen(remote_device_name) == peer_bdname_len
                    && strncmp(peer_bdname, remote_device_name, peer_bdname_len) == 0) {
                    memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
                    /* Have found the target peer device, cancel the previous GAP discover procedure. And go on
                     * dsicovering the SPP service on the peer device */
                    esp_bt_gap_cancel_discovery();
                    if (!is_connected) {
                        esp_spp_start_discovery(peer_bd_addr);
                    }
                }
            }
        }
        break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
        // If the device is not connected, restart the discovery process
        if (!is_connected) {
                esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
            }
        break;
    case ESP_BT_GAP_RMT_SRVCS_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
        break;
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
        break;
    default:
        break;
    }   
}

void app_main(void)
{
    init_led();                             // Initialization of LED diodes

    // Inicializace fronty pro data
    data_queue = xQueueCreate(DATA_QUEUE_SIZE, sizeof(uint8_t) * SPP_DATA_LEN);
    if (data_queue == NULL) {
        ESP_LOGE(SPP_TAG, "Failed to create queue");
        return;
    }

    // Creation of a task for data processing
    xTaskCreate(data_processing_task, "data_processing_task", 2048, NULL, 5, NULL);
        
    esp_err_t ret = ESP_OK;
    char bda_str[18] = {0};

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

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

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;                            // ESP_BT_IO_CAP_NONE without the option of input and entering a PIN (the original value was ESP_BT_IO_CAP_IO and required a PIN)
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
    
#endif

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_spp_cfg_t bt_spp_cfg = {
        .mode = esp_spp_mode,
        .enable_l2cap_ertm = esp_spp_enable_l2cap_ertm,
        .tx_buffer_size = 0, /* Only used for ESP_SPP_MODE_VFS mode */
    };
    if ((ret = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    } 

    ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

    // Initialization of the timer for blinking
    blink_timer = xTimerCreate("BlinkTimer", pdMS_TO_TICKS(250), pdTRUE, (void *)0, blink_timer_callback);  // pdMS_TO_TICKS(250) sets the blinking speed in ms
    xTimerStart(blink_timer, 0); // Start blinking right at the beginning

    // Initialization of the timer for reconnection
    reconnect_timer = xTimerCreate("ReconnectTimer", pdMS_TO_TICKS(10000), pdFALSE, (void *)0, reconnect_timer_callback);

}
