
/*******************************************************************************
 * File Name: app_bt_gatt_handler.c
 *
 * Description: This file contains the task that handles GATT events.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
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
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) SUSTAINED BY YOU OR A THIRD PARTY, HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* ON ARISING IN ANY WAY OUT OF THE USE OF THIS SAMPLE CODE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************/

/*******************************************************************************
 * Header Files
 ******************************************************************************/
#include "inttypes.h"
#include <FreeRTOS.h>
#include <task.h>
#include "wiced_memory.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "app_bt_bonding.h"
#include "app_flash_common.h"
#include "cycfg_gap.h"
#include "app_bt_utils.h"
#include "app_bt_event_handler.h"
#include "app_bt_gatt_handler.h"
#include "app_hw_device.h"
#include "gpio_task.h"
#include "command.h"
#include "uart_task.h"
#include "i2cm_task.h"
#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

#define WRITE_READ 6
#define READ  1
#define WRITE 0

/*******************************************************************************
 * Variable Definitions
 ******************************************************************************/
extern bluetooth_module_state_t bluetooth_module_state;
//Global buffer
uint8_t i2c_hex_char_buffer[BUFFER_SIZE]; //After sending al the I2C frame, the buffer has to be empty or clean
uint8_t i2c_read_buffer[BUFFER_SIZE-7]; // Buffer para leer datos

/*******************************************************************************
 * Function Definitions
 ******************************************************************************/
/**
 * Function Name: app_bt_gatt_event_callback
 *
 * Function Description:
 *   @brief This function handles GATT events from the BT stack.
 *
 *   @param wiced_bt_gatt_evt_t event                : LE GATT event code of one
 *          byte length
 *   @param wiced_bt_gatt_event_data_t *p_event_data : Pointer to LE GATT event
 *                                                    structures
 *
 *   @return wiced_bt_gatt_status_t                  : See possible status
 *           codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_callback(wiced_bt_gatt_evt_t event,
                            wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_ERROR;

    wiced_bt_gatt_attribute_request_t *p_attr_req = &p_event_data->attribute_request;
    /* Call the appropriate callback function based on the GATT event type,
     * and pass the relevant event parameters to the callback function */
    switch (event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            gatt_status = app_bt_gatt_conn_status_cb( &p_event_data->connection_status );
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            gatt_status = app_bt_gatt_req_cb(p_attr_req);

            break;
            /* GATT buffer request, typically sized to max of bearer mtu - 1 */
        case GATT_GET_RESPONSE_BUFFER_EVT:
            p_event_data->buffer_request.buffer.p_app_rsp_buffer =
            app_bt_alloc_buffer(p_event_data->buffer_request.len_requested);
            p_event_data->buffer_request.buffer.p_app_ctxt = (void *)app_bt_free_buffer;
            gatt_status = WICED_BT_GATT_SUCCESS;
            break;
            /* GATT buffer transmitted event,
             * check \ref wiced_bt_gatt_buffer_transmitted_t*/
        case GATT_APP_BUFFER_TRANSMITTED_EVT:
        {
            pfn_free_buffer_t pfn_free =                                       \
            (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function
             * to free it. */
            if (pfn_free)
                pfn_free(p_event_data->buffer_xmitted.p_app_data);

            gatt_status = WICED_BT_GATT_SUCCESS;
        }
            break;


        default:
            gatt_status = WICED_BT_GATT_SUCCESS;
               break;
    }

    return gatt_status;
}

/**
 * Function Name: app_bt_gatt_req_cb
 *
 * Function Description:
 *   @brief This function handles GATT server events from the BT stack.
 *
 * @param p_attr_req             : Pointer to LE GATT connection status
 *
 * @return wiced_bt_gatt_status_t: See possible status codes in
 *                                 wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_req_cb (wiced_bt_gatt_attribute_request_t *p_attr_req)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_ERROR;
    switch ( p_attr_req->opcode )
    {
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
             /* Attribute read request */
            gatt_status =                                                      \
            app_bt_gatt_req_read_handler(p_attr_req->conn_id,
                                                p_attr_req->opcode,
                                                &p_attr_req->data.read_req,
                                                p_attr_req->len_requested);
             break;

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
             /* Attribute write request */
             gatt_status =
             app_bt_gatt_req_write_handler(p_attr_req->conn_id,
                                                 p_attr_req->opcode,
                                                 &p_attr_req->data.write_req,
                                                 p_attr_req->len_requested);

             if ((GATT_REQ_WRITE == p_attr_req->opcode) &&
                 (WICED_BT_GATT_SUCCESS == gatt_status))
             {
                 wiced_bt_gatt_write_req_t *p_write_request = &p_attr_req->data.write_req;
                 wiced_bt_gatt_server_send_write_rsp(p_attr_req->conn_id,
                                                     p_attr_req->opcode,
                                                     p_write_request->handle);
             }
             break;

        case GATT_REQ_MTU:
//        	printf("\r Client MTU: %d\n", p_attr_req->data.remote_mtu);
        	printf("\r Setting MTU: %d\n", CY_BT_MTU_SIZE);
            gatt_status = wiced_bt_gatt_server_send_mtu_rsp(p_attr_req->conn_id,
															p_attr_req->data.remote_mtu,
															CY_BT_MTU_SIZE);
            break;

        case GATT_HANDLE_VALUE_NOTIF:
//             printf("Notification send complete\n");
             break;

        case GATT_REQ_READ_BY_TYPE:
            gatt_status =                                                      \
            app_bt_gatt_req_read_by_type_handler(p_attr_req->conn_id,
                                                       p_attr_req->opcode,
                                                       &p_attr_req->data.read_by_type,
                                                       p_attr_req->len_requested);
            break;

        case GATT_HANDLE_VALUE_CONF:
            {
                printf("Indication Confirmation received \n");
                bluetooth_module_state.flag_indication_sent = FALSE;
            }
             break;

        default:
                printf("ERROR: Unhandled GATT Connection Request case: %d\n",
                       p_attr_req->opcode);
                break;
    }

    return gatt_status;
}

/**
 * Function Name: app_bt_gatt_conn_status_cb
 *
 * Function Description:
 *   @brief This callback function handles connection status changes.
 *
 *   @param wiced_bt_gatt_connection_status_t *p_conn_status :
 *          Pointer to data that has connection details
 *
 *   @return wiced_bt_gatt_status_t                          : See possible
 *    status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_conn_status_cb(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    if (p_conn_status->connected)
    {
        return app_bt_gatt_connection_up(p_conn_status);
    }
    else
    {
        return app_bt_gatt_connection_down(p_conn_status);
    }
}

/**
 * Function Name: app_bt_gatt_req_read_handler
 *
 * Function Description:
 *   @brief This function handles Read Requests received from the client device
 *
 *   @param conn_id              : Connection ID
 *   @param opcode               : LE GATT request type opcode
 *   @param p_read_req           : Pointer to read request containing the handle
 *          to read
 *   @param len_req              : length of data requested
 *
 * @return wiced_bt_gatt_status_t: See possible status codes in
 *                                 wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_req_read_handler( uint16_t conn_id,
                                    wiced_bt_gatt_opcode_t opcode,
                                    wiced_bt_gatt_read_t *p_read_req,
                                    uint16_t len_req)
{

    gatt_db_lookup_table_t  *puAttribute;
    int          attr_len_to_copy;
    uint8_t     *from;
    //uint32_t buffer;
    uint8_t     *buffer = NULL;
    int          to_send;


    puAttribute = app_bt_find_by_handle(p_read_req->handle);
    if (NULL == puAttribute)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    attr_len_to_copy = puAttribute->cur_len;

    printf("read_handler: conn_id:%d Handle:%x offset:%d len:%d\n ",
            conn_id, p_read_req->handle, p_read_req->offset, attr_len_to_copy);

    if (p_read_req->offset >= puAttribute->cur_len)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_OFFSET);
        return WICED_BT_GATT_INVALID_OFFSET;
    }
    //This switch detects from which characteristic is requesting to read the data
    switch(p_read_req->handle){
    	case HDLC_GAP_DEVICE_NAME_VALUE:
    		printf("Characteristic Read Device Name: %s\n", cy_bt_adv_packet_data[1].p_data);
    		break;
    	case HDLC_GAP_APPEARANCE_VALUE:
    		printf("Characteristic Read: GAP Appearance Value\n");
    		break;
    	case HDLC_BLUETOOTH_MODULE_COMMAND_VALUE:
    		printf("Characteristic Read: Command\n");
    		break;
    	case HDLC_BLUETOOTH_MODULE_I2C_RX_DATA_VALUE:
    		printf("Characteristic Read: I2C RX Data\n");
    		buffer = i2c_read_buffer;
    		break;
    	case HDLC_BLUETOOTH_MODULE_UART_TX_DATA_VALUE:
    		printf("Characteristic Read: UART TX Data\n");
    		break;
    	case HDLC_BLUETOOTH_MODULE_UART_RX_DATA_VALUE:
    	    printf("Characteristic Read: UART RX Data\n");
    	    break;
    	case HDLC_BLUETOOTH_MODULE_GPIO_DATA_VALUE:
    		printf("Characteristic Read: GPIO Data\n");
    		buffer = send_GPIO_values(NULL);
    		break;
    	case HDLC_DIS_FIRMWARE_REVISION_STRING_VALUE:
    		buffer = app_dis_firmware_revision_string;
    		printf("Firmware revision: %s\n", buffer);
    		break;
    	default:
    	    printf("Invalid Characteristic\n");
    	    break;
    }

      //Send data of the buffer through Bluetooth
      puAttribute->p_data = buffer;
      to_send = MIN(len_req, attr_len_to_copy - p_read_req->offset);
      //from = ((uint8_t *)puAttribute->p_data) + p_read_req->offset;
      from = puAttribute->p_data + p_read_req->offset;
      /* No need for context, as buff not allocated */
      return wiced_bt_gatt_server_send_read_handle_rsp(conn_id,
                                                       opcode,
                                                       to_send,
                                                       from,
                                                       NULL);

}

/**
 * Function Name: app_bt_gatt_req_write_handler
 *
 * Function Description:
 *   @brief This function handles Write Requests received from the client device
 *
 *   @param conn_id                : Connection ID
 *   @param opcode                 : LE GATT request type opcode
 *   @param p_write_req            : Pointer to LE GATT write request
 *   @param len_req                : length of data requested
 *
 *   @return wiced_bt_gatt_status_t: See possible status codes in
 *                                   wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_req_write_handler(uint16_t conn_id,
                                    wiced_bt_gatt_opcode_t opcode,
                                    wiced_bt_gatt_write_req_t *p_write_req,
                                    uint16_t len_req)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_INVALID_HANDLE;

//      printf("write_handler: conn_id:%d Handle:0x%x offset:%d len:%d\n ",
//           conn_id, p_write_req->handle,
//           p_write_req->offset,
//           p_write_req->val_len );


    /* Attempt to perform the Write Request */

    gatt_status = app_bt_set_value(p_write_req->handle,
                                         p_write_req->p_val,
                                         p_write_req->val_len);

    if(WICED_BT_GATT_SUCCESS != gatt_status)
    {
        printf("WARNING: GATT set attr status 0x%x\n", gatt_status);
    }

    return (gatt_status);
}

/**
 * Function Name : app_bt_gatt_req_read_by_type_handler
 *
 * Function Description :
 *   @brief Process read-by-type request from peer device
 *
 *   @param uint16_t conn_id                       : Connection ID
 *   @param wiced_bt_gatt_opcode_t opcode          : LE GATT request type opcode
 *   @param wiced_bt_gatt_read_by_type_t p_read_req: Pointer to read request
 *          containing the handle to read
 *  @param uint16_t len_requested                  : Length of data requested
 *
 * Return:
 *  wiced_bt_gatt_status_t                         : LE GATT status
 */
wiced_bt_gatt_status_t
app_bt_gatt_req_read_by_type_handler(uint16_t conn_id,
                                           wiced_bt_gatt_opcode_t opcode,
                                           wiced_bt_gatt_read_by_type_t *p_read_req,
                                           uint16_t len_requested)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t last_handle = 0;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = app_bt_alloc_buffer(len_requested);
    uint8_t pair_len = 0;
    int used_len = 0;

    if (NULL == p_rsp)
    {
        printf("No memory, len_requested: %d!!\n",len_requested);
        wiced_bt_gatt_server_send_error_rsp(conn_id,
                                            opcode,
                                            attr_handle,
                                            WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type,
     * between the start and end handles */
    while (WICED_TRUE)
    {
        last_handle = attr_handle;
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle,
                                                        p_read_req->e_handle,
                                                        &p_read_req->uuid);
        if (0 == attr_handle)
            break;

        if ( NULL == (puAttribute = app_bt_find_by_handle(attr_handle)))
        {
            printf("found type but no attribute for %d \n",last_handle);
            wiced_bt_gatt_server_send_error_rsp(conn_id,
                                                opcode,
                                                p_read_req->s_handle,
                                                WICED_BT_GATT_ERR_UNLIKELY);
            app_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        int filled =
        wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used_len,
                                                     len_requested - used_len,
                                                     &pair_len,
                                                     attr_handle,
                                                     puAttribute->cur_len,
                                                     puAttribute->p_data);
        if (0 == filled)
        {
            break;
        }
        used_len += filled;

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (0 == used_len)
    {
        printf("attr not found  start_handle: 0x%04x"
               "end_handle: 0x%04x  Type: 0x%04x\n", p_read_req->s_handle,
                                                       p_read_req->e_handle,
                                                       p_read_req->uuid.uu.uuid16);
        wiced_bt_gatt_server_send_error_rsp(conn_id,
                                            opcode,
                                            p_read_req->s_handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        app_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    return wiced_bt_gatt_server_send_read_by_type_rsp(conn_id,
                                                      opcode,
                                                      pair_len,
                                                      used_len,
                                                      p_rsp,
                                                      (void *)app_bt_free_buffer);
}

/**
 * Function Name: app_bt_gatt_connection_up
 *
 * Function Description:
 *   @brief This function is invoked when connection is established
 *
 *   @param wiced_bt_gatt_connection_status_t *p_status :
 *          Pointer to data that has connection details
 *
 *   @return wiced_bt_gatt_status_t                     : See possible status
 *   codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
	wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
	wiced_result_t result = WICED_BT_SUCCESS;

    printf("Connected to peer device: ");
    print_bd_address(p_status->bd_addr);
    printf("Connection ID '%d' \n", p_status->conn_id);

    /* Update the connection handler.  Save address of the connected device. */
    bluetooth_module_state.conn_id = p_status->conn_id;
    memcpy(bluetooth_module_state.remote_addr, p_status->bd_addr,
           sizeof(wiced_bt_device_address_t));
#ifdef PSOC6_BLE
    /* Refer to Note 2 in Document History section of Readme.md */
    if(pairing_mode == TRUE)
    {
        app_bt_add_devices_to_address_resolution_db();
        pairing_mode = FALSE;
    }
#endif

	/* Trigger 2M PHY update request */
	wiced_bt_ble_phy_preferences_t phy_preferences;

	phy_preferences.rx_phys = BTM_BLE_PREFER_2M_PHY;
	phy_preferences.tx_phys = BTM_BLE_PREFER_2M_PHY;
	memcpy(phy_preferences.remote_bd_addr, bluetooth_module_state.remote_addr,
		   sizeof(wiced_bt_device_address_t));
//	result = wiced_bt_ble_set_phy(&phy_preferences);
//	if (result == WICED_BT_SUCCESS)
//	{
//		printf("Request sent to switch PHY to %dM\n",
//							phy_preferences.tx_phys);
//	}
//	else
//	{
//		printf("PHY switch request failed, result: %d\n", status);
//		CY_ASSERT(0);
//	}

    return WICED_BT_GATT_SUCCESS;
}

/**
 * Function Name: app_bt_gatt_connection_down
 *
 * Function Description:
 *   @brief This function is invoked when connection is disconnected
 *
 *   @param wiced_bt_gatt_connection_status_t *p_status :
 *          Pointer to data that has connection details
 *
 *   @return wiced_bt_gatt_status_t                     : See possible status
 *           codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_connection_down(wiced_bt_gatt_connection_status_t *p_status)
{
    wiced_result_t result;
    UNUSED_VARIABLE(result);
    printf("Peer device disconnected: ");
    print_bd_address(p_status->bd_addr);

    printf("conn_id:%d reason:%s\n", p_status->conn_id,
           get_bt_gatt_disconn_reason_name(p_status->reason));

    /* Reset device */
    printf("RESETTING DEVICE\n");
    __NVIC_SystemReset();

//    /* Resetting the device info */
//    memset(bluetooth_module_state.remote_addr, 0, BD_ADDR_LEN);
//    bluetooth_module_state.conn_id = 0;
//
//    /* Start advertisements after disconnection */
//    printf("Starting Undirected High Advertisement\n");
//    result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
//                                           0,
//                                           NULL);
//    if(result != WICED_BT_SUCCESS)
//    {
//        printf("Start advertisement failed: %d\n", result);
//    }

    return WICED_BT_GATT_SUCCESS;
}

/**
 * Function Name : app_bt_find_by_handle
 *
 * Function Description:
 *   @brief Find attribute description by handle
 *
 *   @param uint16_t handle          : handle to look up
 *
 *   @return gatt_db_lookup_table_t  : pointer containing handle data
 *
 */
gatt_db_lookup_table_t  *app_bt_find_by_handle(uint16_t handle)
{
    int i;
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (handle == app_gatt_db_ext_attr_tbl[i].handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}

/**
 * Function Name: app_bt_set_value
 *
 * Function Description:
 *   @brief This function handles writing to the attribute handle in the GATT
 *   database using the data passed from the BT stack. The value to write is
 *   stored in a buffer whose starting address is passed as one of the
 *   function parameters
 *
 *   @param uint16_t attr_handle : GATT attribute handle
 *   @param uint8_t  p_val       : Pointer to LE GATT write request value
 *   @param uint16_t len         : length of GATT write request
 *
 *
 * @return wiced_bt_gatt_status_t: See possible status codes in
 *                                 wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t app_bt_set_value(uint16_t attr_handle,
                                              uint8_t *p_val,
                                              uint16_t len)
{
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bool_t validLen = WICED_FALSE;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_INVALID_HANDLE;
    uint8_t *p_attr = p_val;

    /* Check for a matching handle entry */
    for (int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Detected a matching handle in external lookup table */
            isHandleInTable = WICED_TRUE;

            /* Check if the buffer has space to store the data */
            validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);

            if (validLen)
            {
                /* Value fits within the supplied buffer; copy over the value */
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                gatt_status = WICED_BT_GATT_SUCCESS;

                switch (attr_handle)
                {
					/* By writing into Characteristic Client Configuration descriptor
					 *  peer can enable or disable notification or indication */
					case HDLD_BLUETOOTH_MODULE_UART_RX_DATA_CLIENT_CHAR_CONFIG:
						   if (len != 2)
						   {
								return WICED_BT_GATT_INVALID_ATTR_LEN;
						   }
						   app_bluetooth_module_uart_rx_data[0] = p_attr[0];
						   clear_uart_buffer();  // Clear the UART RX buffer
						   break;

					case HDLD_BLUETOOTH_MODULE_I2C_RX_DATA_CLIENT_CHAR_CONFIG:
						   if (len != 2)
						   {
								return WICED_BT_GATT_INVALID_ATTR_LEN;
						   }
						   app_bluetooth_module_i2c_data[0] = p_attr[0];
						   break;

					case HDLC_BLUETOOTH_MODULE_COMMAND_VALUE:
						  if (len != 1){
							  return WICED_BT_GATT_INVALID_ATTR_LEN;
						  }

						  app_bluetooth_module_command[0] = p_attr[0];
						  if (app_bluetooth_module_command[0] != 0)
						  {
							  printf("Command Data Received: %x\n", app_bluetooth_module_command[0]);
							  check_command(app_bluetooth_module_command[0]);
						  }
						  break;

					case HDLC_BLUETOOTH_MODULE_I2C_DATA_VALUE:
						 memcpy(i2c_hex_char_buffer, p_attr, len);
						 if(i2c_hex_char_buffer[WRITE_READ] == WRITE){
							 printf("I2C WRITE\n");
							 i2c_send_data(i2c_hex_char_buffer);
						 }else if(i2c_hex_char_buffer[WRITE_READ] == READ){
							 printf("I2C READ\n");
							 i2c_read_data(i2c_hex_char_buffer);
	//						 uint8_t* read_buffer = i2c_read_data(i2c_hex_char_buffer);
	//						 i2c_flag = true;
	//						 app_bt_send_message(read_buffer);
						  }
						 break;

					 case HDLC_BLUETOOTH_MODULE_UART_TX_DATA_VALUE:
						  if (len != 1)
						  {
							  return WICED_BT_GATT_INVALID_ATTR_LEN;
						  }
						  app_bluetooth_module_uart_tx_data[0] = p_attr[0];
						  //printf("UART TX: %s\n", app_bluetooth_module_uart_tx_data);
						  // Sends UART data from Host to FPGA
						  transmit_UART(app_bluetooth_module_uart_tx_data, len);
						  break;

					 case HDLC_BLUETOOTH_MODULE_UART_RX_DATA_VALUE:
//						  if (len != 1)
//						  {
//							   return WICED_BT_GATT_INVALID_ATTR_LEN;
//						  }
//						  app_bluetooth_module_uart_rx_data[0] = p_attr[0];
//						  if (app_bluetooth_module_uart_rx_data[0] != 0)
//						  {
//							  printf("UART Data Received: %x\n", app_bluetooth_module_uart_data[0]);
//							  check_command(app_bluetooth_module_uart_data[0]);
//						  }
						  break;

	//                 case HDLC_BLUETOOTH_MODULE_GPIO_DATA_VALUE:
	//                      if (len != 1)
	//                      {
	//                          return WICED_BT_GATT_INVALID_ATTR_LEN;
	//                      }
	//                      app_bluetooth_module_gpio_data[0] = p_attr[0];
	//                      if (app_bluetooth_module_gpio_data[0] != 0)
	//                       {
	//                          printf("GPIO Data Received\n");
	//                      }
	//                      break;

					  case HDLD_GATT_SERVICE_CHANGED_CLIENT_CHAR_CONFIG:
						   gatt_status = WICED_BT_GATT_SUCCESS;
						   break;

					  default:
						   gatt_status = WICED_BT_GATT_INVALID_HANDLE;
						   break;
						   }
               }
               else
               {
                   /* Value to write does not meet size constraints */
                   gatt_status = WICED_BT_GATT_INVALID_ATTR_LEN;
                }
                break;

  /*
                case HDLD_GATT_SERVICE_CHANGED_CLIENT_CHAR_CONFIG:
                    gatt_status = WICED_BT_GATT_SUCCESS;
                    break;
                    
                default:
                    gatt_status = WICED_BT_GATT_INVALID_HANDLE;
                    break;
                }
            }
            else
            {
                 Value to write does not meet size constraints
                gatt_status = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;*/
        }
    }
    if (!isHandleInTable)
    {
        /* TODO: Add code to read value for handles not contained within
         * generated lookup table. This is a custom logic that depends on the
         * application, and is not used in the current application. If the value
         * for the current handle is successfully written in the below code
         * snippet, then set the result using: res = WICED_BT_GATT_SUCCESS; */
        switch(attr_handle)
        {
            default:
                /* The write operation was not performed for the
                 * indicated handle */
                printf("Write Request to Invalid Handle: 0x%x\n", attr_handle);
                gatt_status = WICED_BT_GATT_WRITE_NOT_PERMIT;
                break;
        }
    }

    return gatt_status;
}

/**
 * Function Name: app_bt_send_uart_message
 *
 * Function Description:
 *   @brief Check if client has registered for notification/indication and send
 *   message if appropriate
 *
 *   @param uint18_t* received_bytes        : pointer to bytes to be sent
 *   @param uint18_t length                 : number of bytes to be sent
 *
 *   @return None
 *
 */
void app_bt_send_uart_message(uint8_t *received_bytes, uint8_t length)
{
//	uart_clear_to_send_flag = false;
	wiced_bt_gatt_status_t status;
//		for (int i = 0; i < length; i++){
//							   printf("%c", received_bytes[i]);
//					   }

	memcpy((char*)app_bluetooth_module_uart_rx_data, (char*)received_bytes, (size_t)length);

//	for (int i = 0; i < length; i++){
//						   printf("%c", app_bluetooth_module_uart_rx_data[i]);
//				   }

	status = wiced_bt_gatt_server_send_notification(bluetooth_module_state.conn_id,
													HDLC_BLUETOOTH_MODULE_UART_RX_DATA_VALUE,
													length,
													app_bluetooth_module_uart_rx_data,
													NULL);



	if (status != CY_RSLT_SUCCESS)
    {
		// https://infineon.github.io/btsdk-docs/BT-SDK/55572-A1_Bluetooth/API/group__wicedbt__gatt.html#ga972f2437538363989cee28019cb06652
		if (status == 34705) // 0x8791
		{
			printf("ERROR: WICED_BT_GATT_CONGESTED\n");
		} else{
			printf("Notification Status: %04X\n", status);
		}
    }
//	memset(app_bluetooth_module_uart_rx_data, '\0', app_bluetooth_module_uart_rx_data_len);
//	uart_clear_to_send_flag = true;
}


/**
 * Function Name: app_bt_free_buffer
 *
 * Function Description:
 *   @brief This function frees up the memory buffer
 *
 *   @param uint8_t *p_data: Pointer to the buffer to be free
 *
 *   @return None
 */
void app_bt_free_buffer(uint8_t *p_buf)
{
    vPortFree(p_buf);
}

/**
 * Function Name: app_bt_alloc_buffer
 *
 * Function Description:
 *   @brief This function allocates a memory buffer.
 *
 *   @param int len: Length to allocate
 *
 *   @return None
 */
void* app_bt_alloc_buffer(int len)
{
    return pvPortMalloc(len);
}
