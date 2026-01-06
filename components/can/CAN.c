/**
 * @section License
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017, Thomas Barth, barth-dev.de
 *               2017, Jaime Breva, jbreva@nayarsystems.com
 *               2025, Updated to use TWAI driver for ESP-IDF v5.5
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "CAN.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "CAN_config.h"

static const char *TAG = "CAN";
static TaskHandle_t rx_task_handle = NULL;

// Convert CAN speed enum to TWAI timing config
static void get_twai_timing(CAN_speed_t speed, twai_timing_config_t *t_config) {
    switch (speed) {
        case CAN_SPEED_100KBPS:
            *t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_100KBITS();
            break;
        case CAN_SPEED_125KBPS:
            *t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
            break;
        case CAN_SPEED_250KBPS:
            *t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
            break;
        case CAN_SPEED_500KBPS:
            *t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
            break;
        case CAN_SPEED_800KBPS:
            *t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_800KBITS();
            break;
        case CAN_SPEED_1000KBPS:
            *t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
            break;
        default:
            *t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
            break;
    }
    
    // Disable triple sampling temporarily for testing
    t_config->triple_sampling = false;
}

// RX task that reads TWAI messages and puts them in the queue
static void twai_rx_task(void *arg) {
    twai_message_t rx_msg;
    CAN_frame_t can_frame;
    
    while (1) {
        // Check for alerts (Bus Off, Error Passive, etc.)
        uint32_t alerts;
        if (twai_read_alerts(&alerts, 0) == ESP_OK) {
            if (alerts & TWAI_ALERT_BUS_OFF) {
                ESP_LOGE(TAG, "Bus Off condition occurred. Initiating recovery...");
                twai_initiate_recovery();
            }
            if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                ESP_LOGI(TAG, "Bus Recovered. Restarting TWAI driver...");
                twai_start();
            }
        }

        // Wait for message to be received
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            // Convert TWAI message to CAN frame format
            can_frame.MsgID = rx_msg.identifier;
            can_frame.FIR.B.DLC = rx_msg.data_length_code;
            can_frame.FIR.B.FF = rx_msg.extd ? CAN_frame_ext : CAN_frame_std;
            can_frame.FIR.B.RTR = rx_msg.rtr ? CAN_RTR : CAN_no_RTR;
            
            // Copy data
            for (int i = 0; i < rx_msg.data_length_code && i < 8; i++) {
                can_frame.data.u8[i] = rx_msg.data[i];
            }
            
            // Send to queue if configured
            if (CAN_cfg.rx_queue != NULL) {
                xQueueSendToBack(CAN_cfg.rx_queue, &can_frame, 0);
            }
        }
    }
}

int CAN_init() {
    // Get TWAI timing configuration based on speed
    twai_timing_config_t t_config;
    get_twai_timing(CAN_cfg.speed, &t_config);
    
    // Configure TWAI general settings
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_cfg.tx_pin_id, 
        (gpio_num_t)CAN_cfg.rx_pin_id, 
        TWAI_MODE_NORMAL
    );
    g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_ERR_PASS | 
                              TWAI_ALERT_BUS_ERROR | TWAI_ALERT_TX_FAILED | TWAI_ALERT_TX_SUCCESS;

    // Configure TWAI filter to accept all messages
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // Install TWAI driver
    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(ret));
        return -1;
    }
    
    // Start TWAI driver
    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(ret));
        twai_driver_uninstall();
        return -1;
    }
    
    ESP_LOGI(TAG, "TWAI driver started on TX:%d RX:%d at %d kbps", 
             CAN_cfg.tx_pin_id, CAN_cfg.rx_pin_id, CAN_cfg.speed);
    
    // Create RX task if queue is configured
    if (CAN_cfg.rx_queue != NULL) {
        xTaskCreate(twai_rx_task, "twai_rx", 4096, NULL, 5, &rx_task_handle);
    }
    
    return 0;
}

int CAN_write_frame(const CAN_frame_t *p_frame) {
    // Convert CAN frame to TWAI message
    twai_message_t tx_msg = {
        .identifier = p_frame->MsgID,
        .data_length_code = p_frame->FIR.B.DLC,
        .extd = (p_frame->FIR.B.FF == CAN_frame_ext) ? 1 : 0,
        .rtr = (p_frame->FIR.B.RTR == CAN_RTR) ? 1 : 0,
        .ss = 0,
        .self = 0,
        .dlc_non_comp = 0,
    };
    
    // Copy data
    for (int i = 0; i < p_frame->FIR.B.DLC && i < 8; i++) {
        tx_msg.data[i] = p_frame->data.u8[i];
    }
    
    // Transmit message with 1 second timeout
    esp_err_t ret = twai_transmit(&tx_msg, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to transmit: %s", esp_err_to_name(ret));
        return -1;
    }
    
    return 0;
}

int CAN_stop() {
    // Stop RX task
    if (rx_task_handle != NULL) {
        vTaskDelete(rx_task_handle);
        rx_task_handle = NULL;
    }
    
    // Stop TWAI driver
    esp_err_t ret = twai_stop();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop TWAI: %s", esp_err_to_name(ret));
    }
    
    // Uninstall TWAI driver
    ret = twai_driver_uninstall();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to uninstall TWAI: %s", esp_err_to_name(ret));
        return -1;
    }
    
    ESP_LOGI(TAG, "TWAI driver stopped");
    return 0;
}

void CAN_print_diagnostics(void) {
    twai_status_info_t status_info;
    
    if (twai_get_status_info(&status_info) == ESP_OK) {
        printf("\n========== CAN BUS DIAGNOSTICS ==========\n");
        printf("State: ");
        switch (status_info.state) {
            case TWAI_STATE_RUNNING:
                printf("RUNNING\n");
                break;
            case TWAI_STATE_BUS_OFF:
                printf("BUS OFF (CRITICAL - Too many errors!)\n");
                break;
            case TWAI_STATE_RECOVERING:
                printf("RECOVERING\n");
                break;
            case TWAI_STATE_STOPPED:
                printf("STOPPED\n");
                break;
            default:
                printf("UNKNOWN\n");
                break;
        }
        
        printf("TX Error Counter: %lu\n", (unsigned long)status_info.tx_error_counter);
        printf("RX Error Counter: %lu\n", (unsigned long)status_info.rx_error_counter);
        printf("Messages in TX Queue: %lu\n", (unsigned long)status_info.msgs_to_tx);
        printf("Messages in RX Queue: %lu\n", (unsigned long)status_info.msgs_to_rx);
        printf("TX Failed Count: %lu\n", (unsigned long)status_info.tx_failed_count);
        printf("RX Missed Count: %lu\n", (unsigned long)status_info.rx_missed_count);
        printf("RX Overrun Count: %lu\n", (unsigned long)status_info.rx_overrun_count);
        printf("Arbitration Lost Count: %lu\n", (unsigned long)status_info.arb_lost_count);
        printf("Bus Error Count: %lu\n", (unsigned long)status_info.bus_error_count);
        
        // Interpret error counters
        printf("\n--- DIAGNOSIS ---\n");
        if (status_info.tx_error_counter > 96) {
            printf("⚠️  HIGH TX ERRORS! Usually means NO ACK from other devices.\n");
            printf("   → Check: 1) Other device is powered and connected\n");
            printf("   → Check: 2) 120Ω termination resistors at BOTH ends\n");
            printf("   → Check: 3) CANH/CANL wiring is correct\n");
        } else if (status_info.tx_error_counter > 0) {
            printf("⚠️  Some TX errors detected (ACK issues).\n");
        } else {
            printf("✓ TX Error Counter: OK\n");
        }
        
        if (status_info.rx_error_counter > 96) {
            printf("⚠️  HIGH RX ERRORS! Check signal quality and termination.\n");
        } else if (status_info.rx_error_counter > 0) {
            printf("⚠️  Some RX errors detected.\n");
        } else {
            printf("✓ RX Error Counter: OK\n");
        }
        
        if (status_info.state == TWAI_STATE_BUS_OFF) {
            printf("\n🔴 BUS OFF STATE - CAN controller has shut down!\n");
            printf("   This means too many consecutive errors occurred.\n");
            printf("   Fix hardware issues, then restart the device.\n");
        }
        
        printf("=========================================\n\n");
    } else {
        printf("ERROR: Could not read CAN status\n");
    }
}

