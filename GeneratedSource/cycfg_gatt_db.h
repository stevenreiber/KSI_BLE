/***************************************************************************//**
* File Name: cycfg_gatt_db.h
*
* Description:
* Definitions for constants used in the device's GATT database and function
* prototypes.
* This file should not be modified. It was automatically generated by
* Bluetooth Configurator 2.60.0.1460
*
********************************************************************************
* Copyright 2024 Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#if !defined(CYCFG_GATT_DB_H)
#define CYCFG_GATT_DB_H

#include "stdint.h"

#define __UUID_SERVICE_GENERIC_ACCESS                            0x1800
#define __UUID_CHARACTERISTIC_DEVICE_NAME                        0x2A00
#define __UUID_CHARACTERISTIC_APPEARANCE                         0x2A01
#define __UUID_SERVICE_GENERIC_ATTRIBUTE                         0x1801
#define __UUID_CHARACTERISTIC_SERVICE_CHANGED                    0x2A05
#define __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION    0x2902
#define __UUID_SERVICE_DEVICE_INFORMATION                        0x180A
#define __UUID_CHARACTERISTIC_FIRMWARE_REVISION_STRING           0x2A26
#define __UUID_SERVICE_BLUETOOTH_MODULE                          0x28, 0x1D, 0xD7, 0x29, 0xB0, 0xB1, 0xEF, 0x94, 0x1C, 0x44, 0xE3, 0x49, 0xE2, 0x07, 0xFD, 0x4C
#define __UUID_CHARACTERISTIC_BLUETOOTH_MODULE_GPIO_DATA         0xD4, 0x14, 0x49, 0x86, 0x7A, 0x75, 0x85, 0x96, 0x5D, 0x4F, 0x7F, 0x72, 0x0F, 0x51, 0x55, 0x7E
#define __UUID_CHARACTERISTIC_BLUETOOTH_MODULE_COMMAND           0x0C, 0xC2, 0x58, 0xCD, 0xD4, 0xE0, 0x51, 0xB2, 0x9E, 0x4E, 0xA8, 0xB9, 0xF3, 0x9E, 0xE6, 0x80
#define __UUID_CHARACTERISTIC_BLUETOOTH_MODULE_I2C_DATA          0x45, 0xEB, 0x98, 0x99, 0x4E, 0x5B, 0x39, 0x9E, 0xD4, 0x40, 0x96, 0x62, 0xF3, 0x26, 0x0C, 0xBF
#define __UUID_CHARACTERISTIC_BLUETOOTH_MODULE_I2C_RX_DATA       0xE5, 0xBA, 0xDA, 0x8B, 0xBF, 0xC4, 0x7C, 0xAA, 0x84, 0x47, 0x68, 0x7E, 0x6F, 0xF3, 0x65, 0x86
#define __UUID_CHARACTERISTIC_BLUETOOTH_MODULE_UART_TX_DATA      0x2D, 0x88, 0xBF, 0x2A, 0x91, 0x59, 0xC7, 0xB8, 0x8B, 0x46, 0x38, 0x97, 0x1C, 0x4F, 0x6D, 0x3A
#define __UUID_CHARACTERISTIC_BLUETOOTH_MODULE_UART_RX_DATA      0x4F, 0x5D, 0xBC, 0x44, 0x09, 0x24, 0x4B, 0xBD, 0xE3, 0x48, 0xEA, 0xF7, 0xC3, 0x43, 0xD3, 0xA3

/* Service Generic Access */
#define HDLS_GAP                                                 0x0001
/* Characteristic Device Name */
#define HDLC_GAP_DEVICE_NAME                                     0x0002
#define HDLC_GAP_DEVICE_NAME_VALUE                               0x0003
/* Characteristic Appearance */
#define HDLC_GAP_APPEARANCE                                      0x0004
#define HDLC_GAP_APPEARANCE_VALUE                                0x0005

/* Service Generic Attribute */
#define HDLS_GATT                                                0x0006
/* Characteristic Service Changed */
#define HDLC_GATT_SERVICE_CHANGED                                0x0007
#define HDLC_GATT_SERVICE_CHANGED_VALUE                          0x0008
/* Descriptor Client Characteristic Configuration */
#define HDLD_GATT_SERVICE_CHANGED_CLIENT_CHAR_CONFIG             0x0009

/* Service Device Information */
#define HDLS_DIS                                                 0x000A
/* Characteristic Firmware Revision String */
#define HDLC_DIS_FIRMWARE_REVISION_STRING                        0x000B
#define HDLC_DIS_FIRMWARE_REVISION_STRING_VALUE                  0x000C

/* Service Bluetooth Module */
#define HDLS_BLUETOOTH_MODULE                                    0x000D
/* Characteristic GPIO Data */
#define HDLC_BLUETOOTH_MODULE_GPIO_DATA                          0x000E
#define HDLC_BLUETOOTH_MODULE_GPIO_DATA_VALUE                    0x000F
/* Characteristic Command */
#define HDLC_BLUETOOTH_MODULE_COMMAND                            0x0010
#define HDLC_BLUETOOTH_MODULE_COMMAND_VALUE                      0x0011
/* Characteristic I2C Data */
#define HDLC_BLUETOOTH_MODULE_I2C_DATA                           0x0012
#define HDLC_BLUETOOTH_MODULE_I2C_DATA_VALUE                     0x0013
/* Characteristic I2C Rx Data */
#define HDLC_BLUETOOTH_MODULE_I2C_RX_DATA                        0x0014
#define HDLC_BLUETOOTH_MODULE_I2C_RX_DATA_VALUE                  0x0015
/* Descriptor Client Characteristic Configuration */
#define HDLD_BLUETOOTH_MODULE_I2C_RX_DATA_CLIENT_CHAR_CONFIG     0x0016
/* Characteristic UART Tx Data */
#define HDLC_BLUETOOTH_MODULE_UART_TX_DATA                       0x0017
#define HDLC_BLUETOOTH_MODULE_UART_TX_DATA_VALUE                 0x0018
/* Characteristic UART Rx Data */
#define HDLC_BLUETOOTH_MODULE_UART_RX_DATA                       0x0019
#define HDLC_BLUETOOTH_MODULE_UART_RX_DATA_VALUE                 0x001A
/* Descriptor Client Characteristic Configuration */
#define HDLD_BLUETOOTH_MODULE_UART_RX_DATA_CLIENT_CHAR_CONFIG    0x001B


/* External Lookup Table Entry */
typedef struct
{
    uint16_t handle;
    uint16_t max_len;
    uint16_t cur_len;
    uint8_t  *p_data;
} gatt_db_lookup_table_t;

/* External definitions */
extern const uint8_t  gatt_database[];
extern const uint16_t gatt_database_len;
extern gatt_db_lookup_table_t app_gatt_db_ext_attr_tbl[];
extern const uint16_t app_gatt_db_ext_attr_tbl_size;
extern uint8_t app_gap_device_name[];
extern const uint16_t app_gap_device_name_len;
extern uint8_t app_gap_appearance[];
extern const uint16_t app_gap_appearance_len;
extern uint8_t app_gatt_service_changed[];
extern const uint16_t app_gatt_service_changed_len;
extern uint8_t app_gatt_service_changed_client_char_config[];
extern const uint16_t app_gatt_service_changed_client_char_config_len;
extern uint8_t app_dis_firmware_revision_string[];
extern const uint16_t app_dis_firmware_revision_string_len;
extern uint8_t app_bluetooth_module_gpio_data[];
extern const uint16_t app_bluetooth_module_gpio_data_len;
extern uint8_t app_bluetooth_module_command[];
extern const uint16_t app_bluetooth_module_command_len;
extern uint8_t app_bluetooth_module_i2c_data[];
extern const uint16_t app_bluetooth_module_i2c_data_len;
extern uint8_t app_bluetooth_module_i2c_rx_data[];
extern const uint16_t app_bluetooth_module_i2c_rx_data_len;
extern uint8_t app_bluetooth_module_i2c_rx_data_client_char_config[];
extern const uint16_t app_bluetooth_module_i2c_rx_data_client_char_config_len;
extern uint8_t app_bluetooth_module_uart_tx_data[];
extern const uint16_t app_bluetooth_module_uart_tx_data_len;
extern uint8_t app_bluetooth_module_uart_rx_data[];
extern const uint16_t app_bluetooth_module_uart_rx_data_len;
extern uint8_t app_bluetooth_module_uart_rx_data_client_char_config[];
extern const uint16_t app_bluetooth_module_uart_rx_data_client_char_config_len;

#endif /* CYCFG_GATT_DB_H */
