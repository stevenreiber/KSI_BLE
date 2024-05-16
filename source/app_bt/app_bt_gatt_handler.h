/*******************************************************************************
 * File Name: app_bt_gatt_handler.h
 *
 * Description: This file is the public interface of app_bt_gatt_handler.c
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

#ifndef SOURCE_APP_BT_APP_BT_GATT_HANDLER_H_
#define SOURCE_APP_BT_APP_BT_GATT_HANDLER_H_


/*******************************************************************************
* Header Files
*******************************************************************************/

#include "wiced_bt_gatt.h"
#include "cycfg_gatt_db.h"

/*******************************************************************************
 * Macro Prototypes
 ******************************************************************************/
#define SEND_UART       0
#define RECEIVE_UART    1

/* This enumeration combines the advertising, connection states from two different
 * callbacks to maintain the status in a single state variable */
typedef enum
{
    APP_BT_ADV_OFF_CONN_OFF,
    APP_BT_ADV_ON_CONN_OFF,
    APP_BT_ADV_OFF_CONN_ON
} app_bt_adv_conn_mode_t;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

wiced_bt_gatt_status_t
app_bt_gatt_callback(wiced_bt_gatt_evt_t event,
                     wiced_bt_gatt_event_data_t *p_data);
wiced_bt_gatt_status_t
app_bt_gatt_req_cb(wiced_bt_gatt_attribute_request_t *p_attr_req);
wiced_bt_gatt_status_t
app_bt_gatt_conn_status_cb(wiced_bt_gatt_connection_status_t *p_conn_status);
wiced_bt_gatt_status_t
app_bt_gatt_req_read_handler(uint16_t conn_id,
                             wiced_bt_gatt_opcode_t opcode,
                             wiced_bt_gatt_read_t *p_read_req,
                             uint16_t len_req);
wiced_bt_gatt_status_t
app_bt_gatt_req_write_handler(uint16_t conn_id,
                              wiced_bt_gatt_opcode_t opcode,
                              wiced_bt_gatt_write_req_t *p_write_req,
                              uint16_t len_req);
wiced_bt_gatt_status_t
app_bt_gatt_req_read_by_type_handler(uint16_t conn_id,
                                     wiced_bt_gatt_opcode_t opcode,
                                     wiced_bt_gatt_read_by_type_t *p_read_req,
                                     uint16_t len_requested);

wiced_bt_gatt_status_t
app_bt_gatt_connection_up(wiced_bt_gatt_connection_status_t *p_status);
wiced_bt_gatt_status_t
app_bt_gatt_connection_down(wiced_bt_gatt_connection_status_t *p_status);
gatt_db_lookup_table_t
*app_bt_find_by_handle(uint16_t handle);
wiced_bt_gatt_status_t
app_bt_set_value(uint16_t attr_handle,
                 uint8_t *p_val,
                 uint16_t len);
void app_bt_free_buffer(uint8_t *p_event_data);
void* app_bt_alloc_buffer(int len);
void app_bt_gatt_increment_notify_value(void);
//void app_bt_send_i2c_message(uint8_t* data);
void app_bt_send_uart_message(uint8_t *received_bytes, uint8_t length);
typedef void(*pfn_free_buffer_t) (uint8_t *);

#endif /* SOURCE_APP_BT_APP_BT_GATT_HANDLER_H_ */
