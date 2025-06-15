#include "can.h"


esp_err_t extract_pid_data(twai_message_t *rx_msg, twai_message_t *tx_msg)
{
    twai_message_t temp;
    TickType_t startTick = xTaskGetTickCount();
     if(twai_transmit(tx_msg, pdMS_TO_TICKS(1000)) != ESP_OK) {
        printf("Failed to transmit message\n");
        return ESP_ERR_INVALID_STATE;
     } 
    while ((xTaskGetTickCount() - startTick) < pdMS_TO_TICKS(5000)) {
        if (twai_receive(&temp, pdMS_TO_TICKS(100)) == ESP_OK) {
            if (temp.data[2] == tx_msg->data[2]) {
                *rx_msg = temp; 
                 vTaskDelay(pdMS_TO_TICKS(100)); // Allow some time for the message to be processed
                return ESP_OK; // Exit loop if message is received
            } 
        }
    }
    printf("Failed to read back message\n");
    return ESP_ERR_TIMEOUT; // Return timeout error if no message received
}

esp_err_t get_air_temp(byte_val *air_temp)
{       
    extract_pid_data(&(air_temp->rx_msg), &(air_temp->tx_msg));
    air_temp->data.signed_data = (int8_t)((uint8_t)((*air_temp).rx_msg.data[3]) - 40);
    return ESP_OK;   
}
