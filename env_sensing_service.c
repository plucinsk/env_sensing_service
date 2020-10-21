/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include <stdint.h>
#include <string.h>
#include "env_sensing_service.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "SEGGER_RTT.h"

void ble_env_sensing_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_os_t * p_env_sensing_service = (ble_os_t *) p_context;
    switch (p_ble_evt -> header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            p_env_sensing_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            p_env_sensing_service->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        default:
            break;
    }
}

static ret_code_t humidity_char_add(ble_os_t * p_env_sensing_service)
{
    // ADD CUSTOM CHARACTERISTIC UUID
    ret_code_t err_code;
    ble_uuid_t char_uuid;

    BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_HUMIDITY_CHARACTERISTIC);

    //ADD R/W PROPERTIES TO CHARACTERISTIC
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;

    //CONFIGURE CCCD METADATA
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    char_md.p_cccd_md = &cccd_md;
    char_md.char_props.notify = 1;

    //CONFIGURE THE ATT METADATA
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    //SET R/W SECURITY LEVEL TO CHARACTERISTIC
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    //CONFIGURE THE CHARACTERISTIC VALUE ATTRIBUTE
    ble_gatts_attr_t attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;

    //SET CHARACTERISTIC VALUE LENGTH
    attr_char_value.max_len = 2;
    attr_char_value.init_len = 2;
    uint8_t value[4] = {0x00, 0x00};
    attr_char_value.p_value = value;

    //ADD CHARACTERISTIC TO THE SERVICE
    err_code = sd_ble_gatts_characteristic_add(p_env_sensing_service->service_handle, &char_md, &attr_char_value, &p_env_sensing_service->char_handles);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

void env_sensing_service_init(ble_os_t * p_env_sensing_service)
{
    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions
    
    ble_uuid_t            service_uuid;

    BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_ENV_SENSING_SERVICE);
        
    // SET SERVICE CONNECTION HANDLE TO DEFAULT VALUE
    p_env_sensing_service -> conn_handle = BLE_CONN_HANDLE_INVALID;
    
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_env_sensing_service->service_handle);

    APP_ERROR_CHECK(err_code);

    err_code = humidity_char_add(p_env_sensing_service);
    APP_ERROR_CHECK(err_code);

    SEGGER_RTT_WriteString(0, "Executing env_sensing_service_init().\n"); // Print message to RTT to the application flow
    SEGGER_RTT_printf(0, "Service UUID: 0x%#04x\n", service_uuid.uuid); // Print service UUID should match definition BLE_UUID_ENV_SENSING_SERVICE
    SEGGER_RTT_printf(0, "Service UUID type: 0x%#02x\n", service_uuid.type); // Print UUID type. Should match BLE_UUID_TYPE_VENDOR_BEGIN. Search for BLE_UUID_TYPES in ble_types.h for more info
    SEGGER_RTT_printf(0, "Service handle: 0x%#04x\n", p_env_sensing_service->service_handle); // Print out the service handle. Should match service handle shown in MCP under Attribute values
}

void humidity_characteristic_update(ble_os_t *p_env_sensing_service, int *humidity)
{
    if (p_env_sensing_service->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint16_t len = 2;
        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));
        
        hvx_params.handle = p_env_sensing_service->char_handles.value_handle;
        hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len = &len;
        hvx_params.p_data = (uint8_t *)humidity;

        sd_ble_gatts_hvx(p_env_sensing_service->conn_handle, &hvx_params);
    }


}
