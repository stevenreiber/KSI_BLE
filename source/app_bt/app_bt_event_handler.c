/*******************************************************************************
 * File Name: app_bt_event_handler.c
 *
 * Description: This file contains the bluetooth event handler that processes
 *              the bluetooth events from host stack.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company)
 *or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 ******************************************************************************/
/*******************************************************************************
 * THIS SAMPLE CODE IS PROVIDED “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL ARROW ELECTRONICS, INC.
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *OR BUSINESS INTERRUPTION) SUSTAINED BY YOU OR A THIRD PARTY, HOWEVER CAUSED
 *AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * ON ARISING IN ANY WAY OUT OF THE USE OF THIS SAMPLE CODE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

/*******************************************************************************
 * Header Files
 ******************************************************************************/
#include <FreeRTOS.h>
#include <stdio.h>
#include <task.h>

#include "app_bt_bonding.h"
#include "app_bt_event_handler.h"
#include "app_bt_gatt_handler.h"
#include "app_bt_utils.h"
#include "app_flash_common.h"
#include "app_hw_device.h"
#include "cycfg_gap.h"
#include "inttypes.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_l2c.h"
#include "wiced_bt_stack.h"
#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

/*******************************************************************************
 * Variable Definitions
 ******************************************************************************/
/* Variable to store connection state information */
// static conn_state_info_t conn_state_info;

/* Holds the global state of the BLE Module application */
bluetooth_module_state_t bluetooth_module_state;

/*Variable to store connection parameter status*/
wiced_bool_t conn_param_status = FALSE;

/* This is the index for the link keys, cccd and privacy mode of the host we are
 * currently bonded to */
uint8_t bondindex = 0;

static uint8_t *get_unique_hash(void);

/*******************************************************************************
 * Function Definitions
 ******************************************************************************/

/**
 * Function Name: app_bt_management_callback
 *
 * Function Description:
 *   @brief This is a Bluetooth stack event handler function to receive
 *   management events from the LE stack and process as per the application.
 *
 *   @param wiced_bt_management_evt_t event             : LE event code of
 *          one byte length
 *   @param wiced_bt_management_evt_data_t *p_event_data: Pointer to LE
 *          management event structures
 *
 *   @return wiced_result_t                             : Error code from
 *           WICED_RESULT_LIST or BT_RESULT_LIST
 *
 */
wiced_result_t app_bt_management_callback(
    wiced_bt_management_evt_t event,
    wiced_bt_management_evt_data_t *p_event_data) {
    wiced_result_t result = WICED_BT_SUCCESS;
    cy_rslt_t rslt;
    wiced_bt_dev_encryption_status_t *p_status;
    wiced_bt_ble_advert_mode_t *p_mode;
    wiced_bt_dev_ble_pairing_info_t *p_info;
    wiced_bt_device_address_t local_bda = {0x00, 0xA0, 0x50, 0x011, 0x44, 0x55};

    printf("Event:%s\n", get_btm_event_name(event));

    switch (event) {
        case BTM_ENABLED_EVT:
            /* Bluetooth Controller and Host Stack Enabled */
            if (WICED_BT_SUCCESS == p_event_data->enabled.status) {
                wiced_bt_set_local_bdaddr(local_bda, BLE_ADDR_PUBLIC);
                wiced_bt_dev_read_local_addr(local_bda);
                printf("Local Bluetooth Address: ");
                print_bd_address(local_bda);

                /* Perform application-specific initialization */
                app_bt_application_init();
            } else {
                printf("Bluetooth enable failed \n");
            }
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap =
                BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data =
                BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req =
                BTM_LE_AUTH_REQ_SC_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size =
                0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys =
                BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK |
                BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys =
                BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK |
                BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_info = &p_event_data->pairing_complete.pairing_complete_info.ble;
            printf(
                "Bluetooth Module, Pairing Complete Reason: %s \n",
                get_bt_smp_status_name((wiced_bt_smp_status_t)p_info->reason));
            /* Update Num of bonded devices and next free slot in slot data*/
            rslt = app_bt_update_slot_data();
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            p_mode = &p_event_data->ble_advert_state_changed;
            printf("Advertisement State Change: %s\n",
                   get_bt_advert_mode_name(*p_mode));
            if (BTM_BLE_ADVERT_OFF == *p_mode) {
                app_bt_adv_stop_handler();
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* save device keys to NVRAM */
            rslt = app_bt_save_device_link_keys(
                &(p_event_data->paired_device_link_keys_update));
            if (CY_RSLT_SUCCESS == rslt) {
                printf("Successfully Bonded to ");
                print_bd_address(
                    p_event_data->paired_device_link_keys_update.bd_addr);
            } else {
                printf("Failed to bond! \n");
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* Paired Device Link Keys Request */
            printf("Paired Device Link keys Request Event for device ");
            print_bd_address(
                (uint8_t *)(p_event_data->paired_device_link_keys_request
                                .bd_addr));

            /* Need to search to see if the BD_ADDR we are
             * looking for is in NVRAM. If not, we return WICED_BT_ERROR
             * and the stack will generate keys and will then call
             * BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT so that they
             * can be stored
             */

            /* Assume the device won't be found.
             * If it is, we will set this back to WICED_BT_SUCCESS */
            result = WICED_BT_ERROR;
            bondindex = app_bt_find_device_in_flash(
                p_event_data->paired_device_link_keys_request.bd_addr);
            if (BOND_INDEX_MAX > bondindex) {
                /* Copy the keys to where the stack wants it */
                memcpy(&(p_event_data->paired_device_link_keys_request),
                       &bond_info.link_keys[bondindex],
                       sizeof(wiced_bt_device_link_keys_t));
                result = WICED_BT_SUCCESS;
            } else {
                printf("Device Link Keys not found in the database! \n");
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Update of local privacy keys - save to NVRAM */
            rslt = app_bt_save_local_identity_key(
                p_event_data->local_identity_keys_update);
            if (CY_RSLT_SUCCESS != rslt) {
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            app_kv_store_init();
            /* Read Local Identity Resolution Keys if present in NVRAM*/
            rslt = app_bt_read_local_identity_keys();
            if (CY_RSLT_SUCCESS == rslt) {
                memcpy(&(p_event_data->local_identity_keys_request),
                       &(identity_keys),
                       sizeof(wiced_bt_local_identity_keys_t));
                print_array(&identity_keys,
                            sizeof(wiced_bt_local_identity_keys_t));
                result = WICED_BT_SUCCESS;
            } else {
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_status = &p_event_data->encryption_status;
            printf("Encryption status changed, BDA:");
            print_bd_address(p_status->bd_addr);
            /* Check and retreive the index of the bond data of the device that
             * got connected */
            /* This call will return BOND_INDEX_MAX if the device is not found
             */
            bondindex = app_bt_find_device_in_flash(
                p_event_data->encryption_status.bd_addr);
            if (bondindex < BOND_INDEX_MAX) {
                app_bt_restore_bond_data();
                app_bt_restore_cccd();
                /* Set CCCD value from the value that was previously saved in
                 * the NVRAM */
                app_bluetooth_module_command[0] = peer_cccd_data[bondindex];
                printf("Bond info present in Flash for device: ");
                print_bd_address(p_event_data->encryption_status.bd_addr);
            } else {
                printf("No Bond info present in Flash for device: ");
                print_bd_address(p_event_data->encryption_status.bd_addr);
                bondindex = 0;
            }
            break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,
                                        WICED_BT_SUCCESS);
            break;

        case BTM_BLE_PHY_UPDATE_EVT:
            bluetooth_module_state.rx_phy =
                p_event_data->ble_phy_update_event.rx_phy;
            bluetooth_module_state.tx_phy =
                p_event_data->ble_phy_update_event.tx_phy;
            printf(
                "Selected RX PHY - %dM\nSelected TX PHY - %dM\nPeer address = ",
                bluetooth_module_state.rx_phy, bluetooth_module_state.tx_phy);
            print_bd_address(bluetooth_module_state.remote_addr);
            /* Send connection interval update if required */
            conn_param_status = wiced_bt_l2cap_update_ble_conn_params(
                bluetooth_module_state.remote_addr, CONNECTION_INTERVAL_P6BLE,
                CONNECTION_INTERVAL_P6BLE + 1, CY_BT_CONN_LATENCY,
                SUPERVISION_TIMEOUT);
            if (TRUE == conn_param_status) {
                printf("Connection parameters update has started \n");
            } else {
                printf("Failed to send connection parameter update\n");
            }
            break;
            //            /* Print the updated BLE physical link*/
            //            printf("Selected TX PHY - %dM\nSelected RX PHY -
            //            %dM\n",
            //                    p_event_data->ble_phy_update_event.tx_phy,
            //                    p_event_data->ble_phy_update_event.rx_phy);
            //            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            /* Connection parameters updated */
            if (WICED_SUCCESS ==
                p_event_data->ble_connection_param_update.status) {
                bluetooth_module_state.conn_interval =
                    (double)((p_event_data->ble_connection_param_update
                                  .conn_interval) *
                             CONN_INTERVAL_MULTIPLIER);
                printf("New connection interval: %f ms\n",
                       bluetooth_module_state.conn_interval);
                //				printf("STO = %d\n",
                //				(p_event_data->ble_connection_param_update.supervision_timeout
                //																*
                //STO_MULTIPLIER));
            } else {
                printf("Connection parameters update failed: %d\n",
                       p_event_data->ble_connection_param_update.status);
            }
            break;
            //            printf("Connection parameter update status:%d, "
            //                   "Connection Interval: %d, "
            //                   "Connection Latency: %d, "
            //                   "Connection Timeout: %d\n",
            //                    p_event_data->ble_connection_param_update.status,
            //                    p_event_data->ble_connection_param_update.conn_interval,
            //                    p_event_data->ble_connection_param_update.conn_latency,
            //                    p_event_data->ble_connection_param_update.supervision_timeout);
            //            break;

        default:
            printf("Unhandled Bluetooth Management Event: 0x%x %s\n", event,
                   get_btm_event_name(event));
            break;
    }
    return result;
}

/**
 * Function Name: hci_trace_cback
 *
 * Function Description:
 *   @brief This callback routes HCI packets to debug uart.
 *
 *   @param wiced_bt_hci_trace_type_t type : HCI trace type
 *   @param uint16_t length : length of p_data
 *   @param uint8_t* p_data : pointer to data
 *
 *   @return None
 *
 */
#ifdef ENABLE_BT_SPY_LOG
void hci_trace_cback(wiced_bt_hci_trace_type_t type, uint16_t length,
                     uint8_t *p_data) {
    cybt_debug_uart_send_hci_trace(type, length, p_data);
}
#endif

/**
 * Function Name: app_bt_application_init
 *
 * Function Description:
 *   @brief This function handles application level initialization tasks and is
 *   called from the BT management callback once the LE stack enabled event
 *   (BTM_ENABLED_EVT) is triggered. This function is executed in the
 *   BTM_ENABLED_EVT management callback.
 *
 *   @param None
 *
 *   @return None
 *
 */
void app_bt_application_init(void) {
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    // Set number of characters and container for the variable name
    const uint8_t var_name_len = get_adv_var_name_length();
    uint8_t *var_name;

#ifdef ENABLE_BT_SPY_LOG
    wiced_bt_dev_register_hci_trace(hci_trace_cback);
#endif

    printf("Bluetooth Module application init\n");

    //    app_bt_interrupt_config();

    app_bt_hw_init();

    if (CY_RSLT_SUCCESS == app_bt_restore_bond_data()) {
        printf("Keys found in NVRAM, add them to Addr Res DB\n");
        /* Load previous paired keys for address resolution */
        app_bt_add_devices_to_address_resolution_db();
    }

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, FALSE);

    // https://community.infineon.com/t5/AIROC-Bluetooth/PSoC6-Change-BLE-advertisement-name-during-run-time/td-p/394357
    var_name = (uint8_t *)malloc(var_name_len);
    var_name = get_unique_hash();
    construct_adv_name(var_name, var_name_len);

    /* Set Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE,
                                            cy_bt_adv_packet_data);

    /* Register with BT stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(app_bt_gatt_callback);
    printf("GATT event Handler registration status: %s \n",
           get_bt_gatt_status_name(gatt_status));

    /* Initialize GATT Database */
    gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
    printf("GATT database initialization status: %s \n",
           get_bt_gatt_status_name(gatt_status));

    /* Start Undirected LE Advertisements on device startup.
     * The corresponding parameters are contained in 'app_bt_cfg.c' */
    result =
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

    /* Failed to start advertisement. Stop program execution */
    if (WICED_BT_SUCCESS != result) {
        printf("Failed to start advertisement! \n");
        CY_ASSERT(0);
    }
}

/**
 * Function Name: app_bt_adv_stop_handler
 *
 * Function Description:
 *   @brief This function handles advertisement stop event.
 *
 *   @param None
 *
 *   @return None
 *
 */
void app_bt_adv_stop_handler(void) {
    wiced_result_t result;

    if ((!bluetooth_module_state.conn_id) && (!pairing_mode)) {
        result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                               0, NULL);
        if (result != WICED_BT_SUCCESS)
            printf("Advertisement start failed :%d\n", result);
    } else {
        printf("Stop Advertisement\n");
    }
    UNUSED_VARIABLE(result);
}

/**
 * Function Name: get_unique_hash
 *
 * Function Description:
 *   @brief This function generates reasonably unique hex bytes from device ID
 * information
 *
 *   @param None
 *
 *   @return Pointer to a char array containing 2 unique(enough) hex bytes
 *
 */
static uint8_t *get_unique_hash(void) {
    uint64_t uniquie_id = Cy_SysLib_GetUniqueId();
    uint16_t hash = (uint16_t)(uniquie_id & 0xFFFF);
    hash ^= (uint16_t)((uniquie_id >> 16) & 0xFFFF);
    hash ^= (uint16_t)((uniquie_id >> 32) & 0xFFFF);
    hash ^= (uint16_t)((uniquie_id >> 48) & 0xFFFF);
    char hash_to_hex[5];
    uint8_t *result;
    result = (uint8_t *)malloc(5);
    sprintf(hash_to_hex, "%04X", hash);
    int i;
    for (i = 0; i < 5; i++) {
        result[i] = hash_to_hex[i];
    }
    result[i] = '\0';  // Truncate char array with null char

    return (uint8_t *)result;  // Return char array
}

/* END OF FILE [] */
