#include "driver/gpio.h"
#include "driver/twai.h"
#include "inttypes.h"

#ifndef CAN_H
#define CAN_H
#define CAN_PID_SENSOR_SETUP_STANDARD  { \
    .extd = 0, \
    .rtr = 0, \
    .ss = 0,  \
    .self = 0, \
    .dlc_non_comp = 0, \
    .identifier = 0x7DF, \
    .data_length_code = 8 \
    } 

typedef union 
    {
        uint8_t data;
        int8_t signed_data;
    } byte_data;   
    

typedef struct {
    twai_message_t tx_msg;     
    twai_message_t rx_msg;      
    byte_data data;
} byte_val;

typedef struct {
    twai_message_t tx_msg;     
    twai_message_t rx_msg;      
    float dec;            
} float_val;
#endif // CAN_H

esp_err_t extract_pid_data(twai_message_t *rx_msg, twai_message_t *tx_msg);

esp_err_t get_air_temp(byte_val *air_temp);
// esp_err_t get_engine_load(float_val *air_temp);
// esp_err_t get_man_pres(byte_val *air_temp);
// esp_err_t get_rpm(float_val *air_temp)