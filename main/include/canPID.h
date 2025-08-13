#ifndef canPID_h
#define canPID_h
#include <common.h>
#include "driver/gpio.h"
#include "driver/twai.h"

#define BOOL int8_t
#define IS_FLOAT 1
#define IS_INT 0

#define CAN_PID_SENSOR_SETUP_STANDARD  \
    .extd = 0, \
    .rtr = 0, \
    .ss = 0,  \
    .self = 0, \
    .dlc_non_comp = 0, \
    .identifier = 0x7DF, \
    .data_length_code = 8,
    

 #define CAN_PID_SENSOR_SETUP_EXT  \
    .extd = 1, \
    .rtr = 0, \
    .ss = 0,  \
    .self = 0, \
    .dlc_non_comp = 0, \
    .identifier = 0x18DB33F1, \
    .data_length_code = 8,

#define CAN_PID_EMPTY_PID(PID)  \
    .name = "Empty PID", \
    .f_data = 0.0f, \
    .PID_index = PID, \
    .f_gen_func = NULL,  \
    .is_available = 1, \
    .is_float = 1, 
    
#define CAN_PID_REQUEST(pid_to_request) (uint8_t[]){0x02, 0x01, pid_to_request, 0x55, 0x55, 0x55, 0x55, 0x55} 
#define CAN_PID_LIST_INT(pid, function, id_name) { .name = id_name, .i_data = 0.0f, .PID_index = pid, .i_gen_func = function, .is_available = 1, .is_float = 0, }
#define CAN_PID_LIST_FLOAT(pid, function, id_name) { .name = id_name, .i_data = 0.0f, .PID_index = pid, .f_gen_func = function, .is_available = 1, .is_float = 1, }
#define PID_LIST_SIZE 200

typedef struct PID_data PID_data;
struct PID_data
{   
    const char *name;  /**< Name of the PID */
    union
    {
        int8_t i_data;
        float f_data;
    };
        uint8_t PID_index;
    union {
        float (*f_gen_func)(uint8_t data[]);
        uint8_t (*i_gen_func)(uint8_t data[]);
    };
        struct 
        {
            uint8_t is_available: 1;   /**< Indicates if the PID is available */
            uint8_t is_float: 1;        /**< Reserved bits for future use */
            uint8_t reserved: 6;        /**< Reserved bits for future use */
        };
};

typedef struct {
    twai_message_t sender_node;
    twai_message_t receiver_node;
    struct {
    uint8_t is_extended: 1;                /**< Extended frame format */
    uint8_t is_set: 1;                   /**< Indicates if the CAN bus is set up */
    uint8_t reserved: 6;
    };
} CAN_Data_handler;


extern twai_timing_config_t t_config;
extern const twai_filter_config_t can_pid_filters[];
extern twai_general_config_t can_pid_general_config[];


esp_err_t CAN_init(CAN_Data_handler *car_settings, twai_timing_config_t *t_config, const twai_filter_config_t *filter_config, twai_general_config_t *general_config);

/**
 * @brief Initilize the TWAI driver and Car handler with the given configurations.
 *
 * @note FF
 *
 * @param[in] handle  TWAI driver handle returned by `twai_driver_install_v2`
 * @param[in] message Message to transmit
 * @param[in] ticks_to_wait   Number of FreeRTOS ticks to block on the TX queue
 *
 * @return
 *      - ESP_OK: Transmission successfully queued/initiated
 *      - ESP_ERR_INVALID_ARG: Arguments are invalid
 *      - ESP_ERR_TIMEOUT: Timed out waiting for space on TX queue
 *      - ESP_FAIL: TX queue is disabled and another message is currently transmitting
 *      - ESP_ERR_INVALID_STATE: TWAI driver is not in running state, or is not installed
 *      - ESP_ERR_NOT_SUPPORTED: Listen Only Mode does not support transmissions
 */


esp_err_t CAN_request(CAN_Data_handler *car_settings, uint8_t *data_send, uint8_t *data_expected, uint8_t mask_size, uint64_t mask, TickType_t timeout);
esp_err_t PID_data_init(PID_data *programed_pids, PID_data ***pid_list, uint8_t *list_size, CAN_Data_handler *car_settings);
esp_err_t CAN_request_pid(CAN_Data_handler *car_settings, PID_data *element, TickType_t timeout);
esp_err_t CAN_loop(CAN_Data_handler *car_settings, PID_data ***pid_list, uint8_t pid_list_count);

esp_err_t CAN_print_all_pids(PID_data ***pid_list, uint8_t pid_list_count);
esp_err_t CAN_find_PID(PID_data ***pid_list, uint8_t pid, uint8_t *ret_index);
esp_err_t ble_create_pid_lut(PID_data ***list, uint8_t list_size, uint8_t *lut);

#endif