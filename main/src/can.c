#include <canPID.h>
#include <common.h>

#include <stdint.h>


esp_err_t CAN_request(CAN_Data_handler *car_settings, uint8_t *data_send, uint8_t *data_expected, uint8_t mask_size, uint64_t mask, TickType_t timeout) {
    
    memcpy(car_settings->sender_node.data, data_send, 8); // Copy the request data into the sender node

    if (twai_transmit(&(car_settings->sender_node), pdMS_TO_TICKS(1000)) != ESP_OK) {
        ESP_LOGE("PID_data_init", "Failed to transmit initial message.");
        return ESP_FAIL;
    }
    ESP_LOGI("CAN_init", "Sending values: %02X, %02X, %02X", car_settings->sender_node.data[0], car_settings->sender_node.data[1],
car_settings->sender_node.data[3]);

    TickType_t startTick = xTaskGetTickCount();

    while (xTaskGetTickCount() - startTick < pdMS_TO_TICKS(timeout)) {
       
        if (twai_receive(&(car_settings->receiver_node), pdMS_TO_TICKS(1000)) != ESP_OK) {
            ESP_LOGE("PID_data_init", "Failed to receive initial message.");
            return ESP_FAIL;
        }
         ESP_LOGI("CAN_init", "Returned values: %02X, %02X, %02X", car_settings->receiver_node.data[0], car_settings->receiver_node.data[1],
car_settings->receiver_node.data[3]);
        // Compare received data with expected data using the mask
        uint64_t actual = 0, expected = 0;
        for (int i = 0; i < mask_size; i++) {
            actual   |= ((uint64_t)car_settings->receiver_node.data[i]) << ((i * 8));
            expected |= ((uint64_t)data_expected[i]) << (i * 8);
        }

        if ((actual & mask) == (expected & mask)) {
            return ESP_OK;
        } else {
            ESP_LOGW("PID_data_init", "Received data doesn't match expected mask.");
        }
    }
    return ESP_ERR_TIMEOUT;  // Timeout if no valid response
}

esp_err_t CAN_request_pid(CAN_Data_handler *car_settings, PID_data *element, TickType_t timeout) {
    
    memcpy(car_settings->sender_node.data, CAN_PID_REQUEST(element->PID_index), 8); // Copy the request data into the sender node

    if (twai_transmit(&(car_settings->sender_node), pdMS_TO_TICKS(1000)) != ESP_OK) {
        ESP_LOGE("PID_data_init", "Failed to transmit initial message.");
        return ESP_FAIL;
    }

    TickType_t startTick = xTaskGetTickCount();
    while (xTaskGetTickCount() - startTick < pdMS_TO_TICKS(timeout)) {

        if (twai_receive(&(car_settings->receiver_node), pdMS_TO_TICKS(1000)) != ESP_OK) {
            ESP_LOGE("PID_data_init", "Failed to receive initial message.");
            return ESP_FAIL;
        }

        // Compare received data with expected data using the mask
        if (element->PID_index == car_settings->receiver_node.data[2]) {
            return ESP_OK;  // Return early if the PID index matches
        } else {
            ESP_LOGW("PID_data_init", "Received PID index does not match expected PID index.");
        }
    }

    return ESP_ERR_TIMEOUT;  // Timeout if no valid response
}


esp_err_t CAN_init(CAN_Data_handler *car_settings, twai_timing_config_t *t_config, const twai_filter_config_t *filter_config, twai_general_config_t *general_config)
{
   
    (*car_settings).sender_node = (twai_message_t){
        CAN_PID_SENSOR_SETUP_STANDARD
        .data = {0x02, 0x01, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55}
    };
    car_settings->is_extended = 0; // Default to standard frame format

    uint32_t alerts;
    uint32_t alerts_to_enable = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED;
   
    for (uint8_t i = 0; i < 2; i++) {

       if (twai_driver_install(general_config, t_config, &filter_config[i]) != ESP_OK) {
        ESP_LOGE("CAN_init", "Failed to install TWAI driver.");
        break;
         }
        else { 
            if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
        ESP_LOGI("CAN_init(alert reconfig)", "Alerts reconfigured");
            } else {
        ESP_LOGE("CAN_init(alert reconfig)", "Failed to reconfigure alerts");
        break;;
        }
        twai_start();
        }
            if (twai_transmit(&(car_settings->sender_node), pdMS_TO_TICKS(1000)) == ESP_OK) {
                int res = twai_read_alerts(&alerts, pdMS_TO_TICKS(2000));
                if(res == ESP_OK) {
                   if (alerts & TWAI_ALERT_TX_SUCCESS) {
                         ESP_LOGI("CAN_init", "Message sent successfully");
                        if ((twai_receive(&(car_settings->receiver_node), pdMS_TO_TICKS(1000)) == ESP_OK) && (car_settings->receiver_node.data[2] == 0x00)) {
                               ESP_LOGI("CAN_init", "Message received successfully");
                               car_settings->is_set = 1; // Set the is_set flag to indicate CAN bus is set up
                               break;
                        }
                        else if (i == 0)  {
                            ESP_LOGI("CAN_init", "Failed to receive message back, TRYING 11bit ID");
                        }             
                        else {
                            ESP_LOGE("CAN_init", "Failed to receive on both attempts, stopping TWAI driver");
                        }    
                   }   
                    else {
                        ESP_LOGE("CAN_init", "Message transmission failed");
                        break;
                    }
                }
            
                else if ((res == ESP_ERR_TIMEOUT) && (i == 0)) {
                  ESP_LOGE("CAN_init", "Timeout while sending message (likely no other device ak), TRYING 11bit ID anyway" );
                } 
                
                else if (res == ESP_ERR_TIMEOUT) {
                    ESP_LOGE("CAN_init", "Timeout while sending message on both attempts");
                    break;  // Return failure if unable to send message
                }

                else {
                    ESP_LOGE("CAN_init", "Error on trasmissions: %s", esp_err_to_name(res));
                    break;  // Return the error if reading alerts fails
                }
                
            }
            else {
                ESP_LOGE("CAN_init", "Failed to qeue message");
                break;
            }

            if (twai_stop() == ESP_OK) {
                                twai_driver_uninstall();
                                car_settings->sender_node = (twai_message_t){CAN_PID_SENSOR_SETUP_EXT .data = {0x02, 0x01, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55}};
                            }
    }
        
if (car_settings->is_set == 0) {
    ESP_LOGE("CAN_init", "Failed to set up CAN bus, teminating program");
    twai_driver_uninstall();  // Uninstall the TWAI driver if setup fails
    return ESP_FAIL;  // Return failure if unable to set up CAN bus
}
else {
car_settings->is_extended = car_settings->sender_node.extd; // Set the is_extended flag based on the message format
ESP_LOGI("CAN_init", "YAYYY, CAN bus initialized successfully with %s frame format", car_settings->is_extended ? "extended" : "standard");
ESP_LOGI("CAN_init", "Returned values: %02X, %02X, %02X", car_settings->receiver_node.data[0], car_settings->receiver_node.data[1],
car_settings->receiver_node.data[3]);
}
return ESP_OK;
}

esp_err_t PID_data_init(PID_data *programed_pids, PID_data ***pid_list, uint8_t *list_size, CAN_Data_handler *car_settings)
{


    memcpy(car_settings->sender_node.data, CAN_PID_REQUEST(0x00), sizeof(CAN_PID_REQUEST(0x00))); // Create data request to find PIDs

    if(CAN_request(car_settings, CAN_PID_REQUEST(0x00), (uint8_t[]){0x00, 0x41, 0x00}, 3, 0b0100000100000000, pdMS_TO_TICKS(3000)) != ESP_OK) {
        ESP_LOGE("PID_data_init", "Failed to request PID data.");
        return ESP_FAIL;  // Return error if request fails
    }

    uint32_t pid_count = 0;
    uint8_t pid_bytes = car_settings->receiver_node.data[0] - 2;  // Get the number of PID bytes from the response
    ESP_LOGI("PID_data_init", "data[0]: %02X", car_settings->receiver_node.data[0]);
    ESP_LOGI("PID_data_init", "data[1]: %02X", car_settings->receiver_node.data[1]);
    for (uint8_t i = 0; i < car_settings->receiver_node.data[0] - 2; i++) {
         ESP_LOGI("PID_data_init", "Loop %d", i);
        pid_count |= car_settings->receiver_node.data[i + 3] << (24 - (i * 8));  // Combine the PID bytes into a single value
        
    }
    
    uint8_t pid_count_alt = __builtin_popcount(pid_count);  // Count the number of set bits in pid_count
    *pid_list = malloc(pid_count_alt * sizeof(PID_data*));  // Allocate memory for the list of PID_data pointers
    ESP_LOGI("PID_data_init", "Found %d PIDs", pid_count_alt);
    *list_size = pid_count_alt; 
    ESP_LOGI("PID_data_init", "Hex of %08lX", pid_count);
   uint8_t index = 0;
   for (uint8_t i = 0; i < pid_bytes*8; i++) {
    ESP_LOGI("PID_data_init", "Iterating bits (%d)", i);
        if (pid_count & (1 << (31 - i))) {
            uint8_t list_inc = 0;
            while(1){
                 if (programed_pids[list_inc].f_gen_func == NULL && programed_pids[list_inc].i_gen_func == NULL) {
                 ESP_LOGE("PID_data_init", "PID has no gen_func, stopping set PIDS");
                 (*pid_list)[index] = malloc(sizeof(PID_data));
                 *((*pid_list)[index]) = (PID_data){CAN_PID_EMPTY_PID(i+1)};  // Set the PID index
                 break;
                  }
                  else if (programed_pids[list_inc].PID_index == i+1) {
                    (*pid_list)[index] = &programed_pids[list_inc];  // Point to the existing PID_data
                    ESP_LOGI("PID_data_init", "FOUND PID %d, setting gen_func", i+1);
                  break;
                  }
                  else {
                    list_inc++;
                  }
                }
            index++;
         }
    }

    return ESP_OK;
}



esp_err_t CAN_loop(CAN_Data_handler *car_settings, PID_data ***pid_list, uint8_t pid_list_count, completed_read_func func_caller) {
    if (car_settings == NULL || pid_list == NULL || pid_list_count == 0) {
        ESP_LOGE("CAN_loop", "Invalid parameters: car_settings or pid_list is NULL.");
        return ESP_ERR_INVALID_ARG;
    }

    for (uint8_t i = 0; i < pid_list_count; i++) {
        // Request the PID data
        if (CAN_request_pid(car_settings, ((*pid_list)[i]), pdMS_TO_TICKS(3000)) != ESP_OK) {
            ESP_LOGE("CAN_loop", "Failed to request PID %d data.", (*pid_list)[i]->PID_index);
           // (*pid_list)[i]->f_data = 3.0f;
            continue;  // Skip to the next PID if the request fails
        }
        else{
            ESP_LOGI("CAN_loop", "Successfully requested PID %d data.", (*pid_list)[i]->PID_index);
            if(func_caller) {
                func_caller((*pid_list)[i]);  // Call the function pointer if it is not NULL
            }
        if ((*pid_list)[i]->f_gen_func != NULL || (*pid_list)[i]->i_gen_func != NULL) {
            switch ((*pid_list)[i]->is_float)
            {
            case 0:
                (*pid_list)[i]->i_data = (uint8_t)((*pid_list)[i]->i_gen_func)(car_settings->receiver_node.data);
                break;
            
            default:
                (*pid_list)[i]->f_data = ((*pid_list)[i]->f_gen_func)(car_settings->receiver_node.data);
                break;
            }
        }
        else {
            if (car_settings->receiver_node.data[0] > 2){
            for (uint8_t j = 0; j < car_settings->receiver_node.data[0]-2 ; j++) {
                (*pid_list)[i]->f_data += car_settings->receiver_node.data[j + 3];  // Copy the data directly if no gen_func is defined
            }
        }
             else{
            ESP_LOGE("Can loop", "Pid has no return bytes (setting to 00)");
            (*pid_list)[i]->f_data = 0.0f;
             }
        }
    }
}
    return ESP_OK;  
}

esp_err_t CAN_print_all_pids(PID_data ***pid_list, uint8_t pid_list_count){
     if (pid_list == NULL || pid_list_count == 0) {
        ESP_LOGE("CAN_loop", "Invalid parameters: car_settings or pid_list is NULL.");
        return ESP_ERR_INVALID_ARG;
    }

    PID_data *pid = NULL;  // Initialize a pointer to hold the current PID_data
    for(uint8_t i = 0; i < pid_list_count; i++) {
     pid = (*pid_list)[i];
if (pid == NULL) {
    ESP_LOGE("CAN_print_all_pids", "PID_data[%p] is NULL", pid);
    return ESP_ERR_INVALID_ARG;  // Return error if PID_data pointer is NULL
}
        // ESP_LOGI("CAN_print_all_pids", "pid_list[%d] = %p", i, (*pid_list)[i]);
        if (pid->f_gen_func == NULL && pid->i_gen_func == NULL) {
           printf("PID %d (%s): %f (has NOT been converted) \n", pid->PID_index, pid->name, pid->f_data);
            continue;  // Skip if the PID_data pointer is NULL
        }
        else {
            
        if (pid->is_float == 1) {
            printf("PID %d (%s): %f (has been converted) \n", pid->PID_index, pid->name, pid->f_data);
        } else {
            // printf("CAN_print_all_pids PID %d: %d (has been converted)", (*pid_list[i])->PID_index, (*pid_list[i])->i_data);
            printf("PID %d (%s): %d (has been converted) \n", pid->PID_index, pid->name, pid->i_data);
        }
    }
    }
    return ESP_OK;  // Return success after printing all PIDs
}

esp_err_t CAN_find_PID(PID_data ***pid_list, uint8_t pid, uint8_t pid_size, uint8_t *ret_index){
    if (pid_list == NULL || pid_size == NULL || ret_index == NULL || pid == 0) {
        ESP_LOGE("CAN_find_PID", "Invalid parameters: pid_list or ret_index is NULL.");
        return ESP_ERR_INVALID_ARG;
    }

    for (uint8_t i = 0; i < pid_size; i++) {
       if( (*pid_list)[i]->PID_index == pid ){
            *ret_index = i;  // Set the index of the found PID
            ESP_LOGI("CAN_find_PID", "Found PID %d at index %d", pid, i);
            return ESP_OK;  // Return success if PID is found
        }
       }  
    return ESP_ERR_NOT_FOUND;  // Return error if PID is not found

}