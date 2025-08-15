/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef GATT_SVR_H
#define GATT_SVR_H

/* Includes */
/* NimBLE GATT APIs */
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"

/* NimBLE GAP APIs */
#include "host/ble_gap.h"
#include "canPID.h"
/* Public function declarations */

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
void gatt_svr_subscribe_cb(struct ble_gap_event *event);
int gatt_svc_init(void);
esp_err_t gen_func(void);
esp_err_t BLE_CAN_find_PID(PID_data ***pid_list, uint8_t pid, uint8_t *ret_index);
esp_err_t BLE_pass_PID(PID_data ***can_pid_list, uint8_t can_list_size); //warning, due to size, this is not copied
esp_err_t BLE_send_PID_notification(PID_data* pid_data);


#endif // GATT_SVR_H
