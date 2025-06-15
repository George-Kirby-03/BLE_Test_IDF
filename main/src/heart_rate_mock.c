/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "common.h"
#include "heart_rate.h"
#include "can.h"

/* Private variables */
// static uint8_t heart_rate;
byte_val air_temp = {
    .tx_msg = CAN_PID_SENSOR_SETUP_STANDARD,
    .tx_msg.data = {0x02, 0x01, 0x05, 0x55, 0x55, 0x55, 0x55, 0x55}, // Request for air temperature
    .rx_msg = {{{0}}},
    .data = {0} 
};

byte_val man_pressure = {
    .tx_msg = CAN_PID_SENSOR_SETUP_STANDARD,
    .tx_msg.data = {0x02, 0x01, 0x0B, 0x55, 0x55, 0x55, 0x55, 0x55}, // Request for air temperature
    .rx_msg = {{{0}}},
     .data = {0} 
};

float_val eng_load = {
    .tx_msg = CAN_PID_SENSOR_SETUP_STANDARD,
    .tx_msg.data = {0x02, 0x01, 0x04, 0x55, 0x55, 0x55, 0x55, 0x55}, // Request for air temperature
    .rx_msg = {{{0}}},
    .dec = 0
};

float_val rpm = {
    .tx_msg = CAN_PID_SENSOR_SETUP_STANDARD,
    .tx_msg.data = {0x02, 0x01, 0x0C, 0x55, 0x55, 0x55, 0x55, 0x55}, // Request for air temperature
    .rx_msg = {{{0}}},
    .dec = 0
};



/* Public functions */
//uint8_t get_heart_rate(void) { return heart_rate; }

//void update_heart_rate(void) { heart_rate = 60 + (uint8_t)(esp_random() % 21); }

