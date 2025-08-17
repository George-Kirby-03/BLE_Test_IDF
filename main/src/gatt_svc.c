/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "gatt_svc.h"
#include "common.h"
#include "heart_rate.h"
#include "led.h"
#include "canPID.h"


/* Private function declarations */
static int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg);


/* Private variables */
/* Heart rate service */

static const ble_uuid128_t heart_rate_svc_uuid = BLE_UUID128_INIT(0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0xab, 0xcd, 0x00); // LSB first

static uint8_t heart_rate_chr_val[2] = {0};
static uint16_t *heart_rate_chr_val_handle;
// static const ble_uuid128_t heart_rate_chr_uuid =  BLE_UUID128_INIT(0xf2, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
 //                    0x78, 0x56, 0x34, 0x12, 0xab, 0xcd, 0x00);


static uint16_t pid_attr_handle;
static uint16_t heart_rate_chr_conn_handle = 0;
static bool heart_rate_chr_conn_handle_inited = false;
static bool heart_rate_notifiy_status = false;
static bool heart_rate_ind_status = false;


/* GATT services table */


/* Get PID_lists*/
static uint8_t list_size;
static PID_data **pid_list = NULL;

esp_err_t BLE_pass_PID(PID_data ***can_pid_list, uint8_t can_list_size){
    list_size = can_list_size;
    pid_list = *can_pid_list;
    return ESP_OK;
}

typedef struct {
    uint16_t handle; /**< Attribute handle */
    struct 
        {
            uint8_t is_notify: 1;   /**< Indicates if the PID is available */
            uint8_t is_indicate: 1;        /**< Reserved bits for future use */
            uint8_t reserved: 6;        /**< Reserved bits for future use */
        };
} list_uuid_info_t;

static struct ble_gatt_chr_def* uids = NULL;
static list_uuid_info_t* attr_handles = NULL;

esp_err_t gen_func(void){
    struct ble_gatt_chr_def *chr_list = calloc((list_size + 1), sizeof(struct ble_gatt_chr_def));
    heart_rate_chr_val_handle = calloc((list_size + 1), sizeof(uint16_t));
    attr_handles = calloc(list_size, sizeof(list_uuid_info_t));
    heart_rate_chr_val_handle[list_size] = 0;    
    if (chr_list == NULL) {
        ESP_LOGE("gen_func", "Failed to allocate memory for characteristic list.");
        return ESP_FAIL;
    }

    for (uint8_t i = 0; i < list_size; i++) {
       // ble_uuid16_t uuid = BLE_UUID16_INIT((*list)[i]->PID_index);
        ble_uuid16_t *uuid = malloc(sizeof(ble_uuid16_t));
        if (uuid == NULL) {
            ESP_LOGE("gen_func", "Failed to allocate memory for UUID.");
            free(chr_list);  
            return ESP_FAIL;
        }
        memcpy(uuid, &(ble_uuid16_t)BLE_UUID16_INIT((pid_list)[i]->PID_index), sizeof(ble_uuid16_t));
        chr_list[i] = (struct ble_gatt_chr_def){
            .uuid = &uuid->u,
            .access_cb = heart_rate_chr_access,
            .arg = (void*)(uint32_t)i,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            .min_key_size = 0,
            .val_handle = &heart_rate_chr_val_handle[i],
            .cpfd = NULL
        };
    }
    chr_list[list_size] = (struct ble_gatt_chr_def){0}; // Null-terminate the list
    uids = chr_list;
    return ESP_OK;
}


static struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &heart_rate_svc_uuid.u,
        .characteristics = NULL,
    },
    {0}
};



/* Private functions */
static int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg) {
    /* Local variables */
    int rc;
                                    
    /* Handle access events */
    /* Note: Heart rate characteristic is read only */
    switch (ctxt->op) {

    /* Read characteristic event */
    case BLE_GATT_ACCESS_OP_READ_CHR:
        /* Verify connection handle */
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d",
                     conn_handle, attr_handle);
        } else {
           // ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d",
            //         attr_handle);
        }

        /* Verify attribute handle */
        /*
        uint8_t inc = 0;
        while(1)
        {   
            if(heart_rate_chr_val_handle[inc] == attr_handle){
                break;
               }
            else if(heart_rate_chr_val_handle[inc] == 0){
                return BLE_ATT_ERR_ATTR_NOT_FOUND;
            }
            else{
              inc ++;
            }
        } */

        uint32_t index = (uint32_t)arg;
        ESP_LOGI(TAG, "sENDING DATA of PID=%d value=%f",
                     pid_list[index]->PID_index, pid_list[index]->f_data);

        int data = (int)pid_list[index]->f_data;             
        ESP_LOGI(TAG, "Data=%d size=%d float? %d",
                     data, sizeof(data), pid_list[index]->is_float );
        switch (pid_list[index]->is_float)
        {
        case 1:
            
            rc = os_mbuf_append(ctxt->om, &data,
                                sizeof(data));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            break;
        
        case 0:
             rc = os_mbuf_append(ctxt->om, &pid_list[index]->i_data,
                                sizeof(pid_list[index]->i_data));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            break;
        }
        

        goto error;

    /* Unknown event */
    default:
        goto error;
    }

error:
    ESP_LOGE(
        TAG,
        "unexpected access operation to heart rate characteristic, opcode: %d",
        ctxt->op);
    return BLE_ATT_ERR_UNLIKELY;
} 


/* Public functions */
esp_err_t BLE_send_PID_notification(PID_data* pid_data) {
    if (attr_handles[pid_data->PID_index].is_notify) {
        int rc = ble_gatts_notify(heart_rate_chr_conn_handle,attr_handles[pid_data->PID_index].handle);
        if (rc != 0) {
             ESP_LOGW("HRM", "Notify failed: %d", rc);
                return ESP_FAIL;
        }

        ESP_LOGI(TAG, "heart rate notifiaction sent!, PID: %d", attr_handles[pid_data->PID_index].handle);
    }
    return ESP_OK;
}


/*
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event
 */
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    /* Local variables */
    char buf[BLE_UUID_STR_LEN];
    uint8_t index = 0;
    /* Handle GATT attributes register events */
    switch (ctxt->op) {

    /* Service register event */
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGI(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);

        break;

    /* Characteristic register event */
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGI(TAG,
                 "registering characteristic %s with "
                 "def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        if (CAN_find_PID(&pid_list, ble_uuid_u16(ctxt->chr.chr_def->uuid), list_size, &index) == ESP_OK) {
            ESP_LOGI(TAG, "Found PID %d at index %d", ble_uuid_u16(ctxt->chr.chr_def->uuid), index);
            attr_handles[index].handle = ctxt->chr.val_handle; // Store the handle
        } else {
            ESP_LOGE(TAG, "PID not found for UUID %s", ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf));
        }        
        break;

    /* Descriptor register event */
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGI(TAG, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    /* Unknown event */
    default:
        assert(0);
        break;
    }
}

/*
 *  GATT server subscribe event callback
 *      1. Update heart rate subscription status
 */

void gatt_svr_subscribe_cb(struct ble_gap_event *event) {
    /* Check connection handle */
    if (event->subscribe.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle);
    } else {
        ESP_LOGI(TAG, "subscribe 76by nimble stack; attr_handle=%d",
                 event->subscribe.attr_handle);
    }
    /* Check attribute handle */
 //   if (event->subscribe.attr_handle == heart_rate_chr_val_handle) {
        /* Update heart rate subscription status */
        for(uint8_t i = 0; i < list_size; i++) {
            if (attr_handles[i].handle == event->subscribe.attr_handle) {
                ESP_LOGI(TAG, "Found PID %d at index %d", attr_handles[i].handle, i);
                attr_handles[i].is_notify = event->subscribe.cur_notify;
                attr_handles[i].is_indicate = event->subscribe.cur_indicate;    
                break;
            }
        }
       // pid_attr_handle = event->subscribe.attr_handle;
      //  heart_rate_chr_conn_handle = event->subscribe.conn_handle;
     //   heart_rate_chr_conn_handle_inited = true;
     //   heart_rate_ind_status = event->subscribe.cur_indicate;
     //   heart_rate_notifiy_status = event->subscribe.cur_notify;
  //  }
     
}

/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server
 */
int gatt_svc_init(void) {
    /* Local variables */
    int rc;

    /* Adding charactersitcs to svc struct*/
    gatt_svr_svcs[0].characteristics = uids;

    /* 1. GATT service initialization */
    ble_svc_gatt_init();

    /* 2. Update GATT services counter */
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    /* 3. Add GATT services */
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}
