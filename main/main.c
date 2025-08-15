/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "common.h"
#include "gap.h"
#include "gatt_svc.h"
#include "heart_rate.h"
#include "esp_mac.h"
#include "led.h"
#include "canPID.h"

/*CAN global stoage variables*/
#include "funcPID.h"  // Include the PID functions 
static CAN_Data_handler car_settings;
static uint8_t list_size;
static PID_data **pid_list = NULL;
PID_data test[] = 
        {   
            CAN_PID_LIST_INT(4, Ad255, "Calculated engine load"),
            CAN_PID_LIST_FLOAT(5, Am40, "Engine coolant temperature"),
            CAN_PID_LIST_FLOAT(6, A128m100, "Short term fuel trim (STFT)—Bank 1"),
            CAN_PID_LIST_FLOAT(7, A128m100, "Short term fuel trim (STFT)—Bank 2"),
            CAN_PID_LIST_FLOAT(8, A128m100, "Short term fuel trim (STFT)—Bank 3"),
            CAN_PID_LIST_FLOAT(9, A128m100, "Short term fuel trim (STFT)—Bank 4"),
            CAN_PID_LIST_FLOAT(10, A3, "Fuel pressure (gauge pressure)"),
            CAN_PID_LIST_INT(11, A, "Intake manifold absolute pressure"),
            CAN_PID_LIST_FLOAT(12, A256pBd4, "Engine Speed"),
            CAN_PID_LIST_INT(13, A, "Vehicle speed"),
            CAN_PID_LIST_FLOAT(14, Ad2m64, "Timing advance"),
            CAN_PID_LIST_FLOAT(15, Am40, "Intake air temperature"),
            CAN_PID_LIST_FLOAT(16, A256pBd100, "Mass air flow rate"),
            CAN_PID_LIST_INT(17, Ad255, "Throttle position"),
            {0}
        };
const twai_filter_config_t can_pid_filters[] = {
    {
        .acceptance_code = 0x7e8 << 21,
        .acceptance_mask = ~(0x7ff << 21),
        .single_filter = 1
    },
    {
        .acceptance_code = 0x18daf110 << 3,
        .acceptance_mask = (0x1FFFFFFF << 3),
        .single_filter = 1,
    }
};
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
twai_general_config_t pid_general_config =  TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_NORMAL);


/* Library function declarations */
void ble_store_config_init(void);

/* Private function declarations */
static void on_stack_reset(int reason);
static void on_stack_sync(void);
static void nimble_host_config_init(void);
static void nimble_host_task(void *param);

/* Private functions */
/*
 *  Stack event callback functions
 *      - on_stack_reset is called when host resets BLE stack due to errors
 *      - on_stack_sync is called when host has synced with controller
 */
static void on_stack_reset(int reason) {
    /* On reset, print reset reason to console */
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_stack_sync(void) {
    /* On stack sync, do advertising initialization */
    adv_init();
}

static void nimble_host_config_init(void) {
    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Store host configuration */
    ble_store_config_init();
}

static void nimble_host_task(void *param) {
    /* Task entry log */
    ESP_LOGI(TAG, "nimble host task has been started!");

    /* This function won't return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up at exit */
    vTaskDelete(NULL);
}

static void heart_rate_task(void *param) {
    /* Task entry log */
    ESP_LOGI(TAG, "heart rate task has been started!");

    /* Loop forever */
    while (1) {
        /* Update heart rate value every 1 second */
        // update_heart_rate();
         if (CAN_loop(&car_settings, &pid_list, list_size, BLE_send_PID_notification) != ESP_OK) {
        ESP_LOGE("app_main", "CAN loop failed");
        break;
    }
        CAN_print_all_pids(&pid_list, list_size);
       // ESP_LOGI(TAG, "heart rate updated to %d", get_heart_rate());
        /* Send heart rate indication if enabled  (its a notification hereeee)*/ 
     
        /* Sleep */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    /* Clean up at exit */
    vTaskDelete(NULL);
}



void app_main(void) {
    /* Local variables */
    int rc;
    esp_err_t ret;
    /* Start CAN*/
    if (CAN_init(&car_settings, &t_config, can_pid_filters, &pid_general_config) == ESP_OK) {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        return;
    }
     if (PID_data_init(test, &pid_list, &list_size, &car_settings) == ESP_OK) {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        return;
    }

    BLE_pass_PID(&pid_list, list_size);

    /*
     * NVS flash initialization
     * Dependency of BLE stack to store configurations
     */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nvs flash, error code: %d ", ret);
        return;
    }

    /* NimBLE stack initialization */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nimble stack, error code: %d ",
                 ret);
        return;
    }

    /* GAP service initialization */
    rc = gap_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initialize GAP service, error code: %d", rc);
        return;
    }

    /* Characteristics initialization */
    rc = gen_func();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initilise GATT srv characteristics, error code: %d", rc);
        return;
    }

    /* GATT server initialization */
    rc = gatt_svc_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initialize GATT server, error code: %d", rc);
        return;
    }

    /* NimBLE host configuration initialization */
    nimble_host_config_init();

    /* Start NimBLE host task thread and return */
    xTaskCreatePinnedToCore(nimble_host_task, "NimBLE Host", 5*1024, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(heart_rate_task, "Heart Rate", 4*1024, NULL, 5, NULL, 0);
    return;
}
