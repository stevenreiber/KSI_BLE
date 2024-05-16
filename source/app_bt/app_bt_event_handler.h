/*******************************************************************************
 * File Name: app_bt_event_handler.h
 *
 * Description: This file is the public interface of app_bt_event_handler.c
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

#ifndef SOURCE_APP_BT_APP_BT_EVENT_HANDLER_H_
#define SOURCE_APP_BT_APP_BT_EVENT_HANDLER_H_
#define CONNECTION_INTERVAL_P6BLE (6)
#define SUPERVISION_TIMEOUT (1000)
#define STO_MULTIPLIER (10)
#define CONN_INTERVAL_MULTIPLIER (1.25f)

/*******************************************************************************
 * Structures
 *******************************************************************************/
typedef struct {
    wiced_bt_device_address_t remote_addr;  // remote peer device address
    uint32_t timer_count_s;   // count of sec since the start of seconds timer
    uint32_t timer_count_ms;  // count of ms since the start of ms timer
    uint16_t conn_id;         // connection ID of the peer
    uint16_t peer_mtu;        // peer MTU
    uint8_t
        flag_indication_sent;  // to store the state of indication confirmation
    uint8_t num_to_send;  // number of messages to send. Incremented on each
                          // button interrupt
    double conn_interval; /* connection interval negotiated */
    wiced_bt_ble_host_phy_preferences_t rx_phy; /* RX PHY selected */
    wiced_bt_ble_host_phy_preferences_t tx_phy; /* TX PHY selected */
} bluetooth_module_state_t;

// typedef struct
//{
//    wiced_bt_device_address_t             remote_addr;   /* remote peer device
//    address */ uint16_t                              conn_id;       /*
//    connection ID referenced by the stack */ uint16_t mtu;           /* MTU
//    exchanged after connection */ double conn_interval; /* connection interval
//    negotiated */ wiced_bt_ble_host_phy_preferences_t   rx_phy;        /* RX
//    PHY selected */ wiced_bt_ble_host_phy_preferences_t   tx_phy;        /* TX
//    PHY selected */
//
//} conn_state_info_t;

/*******************************************************************************
 * Variable Definitions
 ******************************************************************************/

extern uint8_t bondindex;
/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

wiced_result_t app_bt_management_callback(
    wiced_bt_management_evt_t event,
    wiced_bt_management_evt_data_t *p_event_data);

void app_bt_application_init(void);

void app_bt_adv_stop_handler(void);

#endif /* SOURCE_APP_BT_APP_BT_EVENT_HANDLER_H_ */
