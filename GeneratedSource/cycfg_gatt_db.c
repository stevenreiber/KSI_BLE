/******************************************************************************
* File Name: cycfg_gatt_db.c
*
* Description:
* BLE device's GATT database and device configuration.
* This file should not be modified. It was automatically generated by
* Bluetooth Configurator 2.60.0.1460
*
*******************************************************************************
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
******************************************************************************/

#include "cycfg_gatt_db.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"

/******************************************************************************
* GATT server definitions
******************************************************************************/

const uint8_t gatt_database[] = 
{
    /* Primary Service: Generic Access */
    PRIMARY_SERVICE_UUID16 (
            HDLS_GAP,
            __UUID_SERVICE_GENERIC_ACCESS),
        /* Characteristic: Device Name */
        CHARACTERISTIC_UUID16 (
                HDLC_GAP_DEVICE_NAME,
                HDLC_GAP_DEVICE_NAME_VALUE,
                __UUID_CHARACTERISTIC_DEVICE_NAME,
                GATTDB_CHAR_PROP_READ,
                GATTDB_PERM_READABLE),
        /* Characteristic: Appearance */
        CHARACTERISTIC_UUID16 (
                HDLC_GAP_APPEARANCE,
                HDLC_GAP_APPEARANCE_VALUE,
                __UUID_CHARACTERISTIC_APPEARANCE,
                GATTDB_CHAR_PROP_READ,
                GATTDB_PERM_READABLE),

    /* Primary Service: Generic Attribute */
    PRIMARY_SERVICE_UUID16 (
            HDLS_GATT,
            __UUID_SERVICE_GENERIC_ATTRIBUTE),
        /* Characteristic: Service Changed */
        CHARACTERISTIC_UUID16 (
                HDLC_GATT_SERVICE_CHANGED,
                HDLC_GATT_SERVICE_CHANGED_VALUE,
                __UUID_CHARACTERISTIC_SERVICE_CHANGED,
                GATTDB_CHAR_PROP_INDICATE,
                GATTDB_PERM_NONE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (
                    HDLD_GATT_SERVICE_CHANGED_CLIENT_CHAR_CONFIG,
                    __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),

    /* Primary Service: Device Information */
    PRIMARY_SERVICE_UUID16 (
            HDLS_DIS,
            __UUID_SERVICE_DEVICE_INFORMATION),
        /* Characteristic: Firmware Revision String */
        CHARACTERISTIC_UUID16 (
                HDLC_DIS_FIRMWARE_REVISION_STRING,
                HDLC_DIS_FIRMWARE_REVISION_STRING_VALUE,
                __UUID_CHARACTERISTIC_FIRMWARE_REVISION_STRING,
                GATTDB_CHAR_PROP_READ,
                GATTDB_PERM_READABLE),

    /* Primary Service: Bluetooth Module */
    PRIMARY_SERVICE_UUID128 (
            HDLS_BLUETOOTH_MODULE,
            __UUID_SERVICE_BLUETOOTH_MODULE),
        /* Characteristic: GPIO Data */
        CHARACTERISTIC_UUID128 (
                HDLC_BLUETOOTH_MODULE_GPIO_DATA,
                HDLC_BLUETOOTH_MODULE_GPIO_DATA_VALUE,
                __UUID_CHARACTERISTIC_BLUETOOTH_MODULE_GPIO_DATA,
                GATTDB_CHAR_PROP_READ,
                GATTDB_PERM_READABLE),
        /* Characteristic: Command */
        CHARACTERISTIC_UUID128_WRITABLE (
                HDLC_BLUETOOTH_MODULE_COMMAND,
                HDLC_BLUETOOTH_MODULE_COMMAND_VALUE,
                __UUID_CHARACTERISTIC_BLUETOOTH_MODULE_COMMAND,
                GATTDB_CHAR_PROP_WRITE,
                GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD),
        /* Characteristic: I2C Data */
        CHARACTERISTIC_UUID128_WRITABLE (
                HDLC_BLUETOOTH_MODULE_I2C_DATA,
                HDLC_BLUETOOTH_MODULE_I2C_DATA_VALUE,
                __UUID_CHARACTERISTIC_BLUETOOTH_MODULE_I2C_DATA,
                GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
                GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD),
        /* Characteristic: I2C Rx Data */
        CHARACTERISTIC_UUID128 (
                HDLC_BLUETOOTH_MODULE_I2C_RX_DATA,
                HDLC_BLUETOOTH_MODULE_I2C_RX_DATA_VALUE,
                __UUID_CHARACTERISTIC_BLUETOOTH_MODULE_I2C_RX_DATA,
                GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY | GATTDB_CHAR_PROP_INDICATE,
                GATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (
                    HDLD_BLUETOOTH_MODULE_I2C_RX_DATA_CLIENT_CHAR_CONFIG,
                    __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                    GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD),
        /* Characteristic: UART Tx Data */
        CHARACTERISTIC_UUID128_WRITABLE (
                HDLC_BLUETOOTH_MODULE_UART_TX_DATA,
                HDLC_BLUETOOTH_MODULE_UART_TX_DATA_VALUE,
                __UUID_CHARACTERISTIC_BLUETOOTH_MODULE_UART_TX_DATA,
                GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
                GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD),
        /* Characteristic: UART Rx Data */
        CHARACTERISTIC_UUID128 (
                HDLC_BLUETOOTH_MODULE_UART_RX_DATA,
                HDLC_BLUETOOTH_MODULE_UART_RX_DATA_VALUE,
                __UUID_CHARACTERISTIC_BLUETOOTH_MODULE_UART_RX_DATA,
                GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY | GATTDB_CHAR_PROP_INDICATE,
                GATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (
                    HDLD_BLUETOOTH_MODULE_UART_RX_DATA_CLIENT_CHAR_CONFIG,
                    __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD),
};

/* Length of the GATT database */
const uint16_t gatt_database_len = sizeof(gatt_database);

/******************************************************************************
* GATT Initial Value Arrays
******************************************************************************/
 
uint8_t app_gap_device_name[] = {'K', 'S', 'I', '-', 'D', 'B', 'L', 'E', '-', '\0',};
uint8_t app_gap_appearance[] = {0x00, 0x00,};
uint8_t app_gatt_service_changed[] = {0x01, 0x00, 0x01, 0x00,};
uint8_t app_gatt_service_changed_client_char_config[] = {0x00, 0x00,};
uint8_t app_dis_firmware_revision_string[] = {'1', '.', '1', '.', '0',};
uint8_t app_bluetooth_module_gpio_data[] = {0x00, 0x00, 0x00, 0x00,};
uint8_t app_bluetooth_module_command[] = {'0',};
uint8_t app_bluetooth_module_i2c_data[] = {'0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0',};
uint8_t app_bluetooth_module_i2c_rx_data[] = {'0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0',};
uint8_t app_bluetooth_module_i2c_rx_data_client_char_config[] = {0x00, 0x00,};
uint8_t app_bluetooth_module_uart_tx_data[] = {'0',};
uint8_t app_bluetooth_module_uart_rx_data[] = {'0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
    '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',};
uint8_t app_bluetooth_module_uart_rx_data_client_char_config[] = {0x00, 0x00,};
 
/******************************************************************************
* GATT Lookup Table
******************************************************************************/
 
gatt_db_lookup_table_t app_gatt_db_ext_attr_tbl[] =
{
    {
        HDLC_GAP_DEVICE_NAME_VALUE,                            /* attribute handle */
        9,                                                     /* maxlen */
        9,                                                     /* curlen */
        app_gap_device_name,                                   /* attribute data */
    },
    {
        HDLC_GAP_APPEARANCE_VALUE,                             /* attribute handle */
        2,                                                     /* maxlen */
        2,                                                     /* curlen */
        app_gap_appearance,                                    /* attribute data */
    },
    {
        HDLC_GATT_SERVICE_CHANGED_VALUE,                       /* attribute handle */
        4,                                                     /* maxlen */
        4,                                                     /* curlen */
        app_gatt_service_changed,                              /* attribute data */
    },
    {
        HDLD_GATT_SERVICE_CHANGED_CLIENT_CHAR_CONFIG,          /* attribute handle */
        2,                                                     /* maxlen */
        2,                                                     /* curlen */
        app_gatt_service_changed_client_char_config,           /* attribute data */
    },
    {
        HDLC_DIS_FIRMWARE_REVISION_STRING_VALUE,               /* attribute handle */
        5,                                                     /* maxlen */
        5,                                                     /* curlen */
        app_dis_firmware_revision_string,                      /* attribute data */
    },
    {
        HDLC_BLUETOOTH_MODULE_GPIO_DATA_VALUE,                 /* attribute handle */
        4,                                                     /* maxlen */
        4,                                                     /* curlen */
        app_bluetooth_module_gpio_data,                        /* attribute data */
    },
    {
        HDLC_BLUETOOTH_MODULE_COMMAND_VALUE,                   /* attribute handle */
        1,                                                     /* maxlen */
        1,                                                     /* curlen */
        app_bluetooth_module_command,                          /* attribute data */
    },
    {
        HDLC_BLUETOOTH_MODULE_I2C_DATA_VALUE,                  /* attribute handle */
        207,                                                   /* maxlen */
        207,                                                   /* curlen */
        app_bluetooth_module_i2c_data,                         /* attribute data */
    },
    {
        HDLC_BLUETOOTH_MODULE_I2C_RX_DATA_VALUE,               /* attribute handle */
        200,                                                   /* maxlen */
        200,                                                   /* curlen */
        app_bluetooth_module_i2c_rx_data,                      /* attribute data */
    },
    {
        HDLD_BLUETOOTH_MODULE_I2C_RX_DATA_CLIENT_CHAR_CONFIG,  /* attribute handle */
        2,                                                     /* maxlen */
        2,                                                     /* curlen */
        app_bluetooth_module_i2c_rx_data_client_char_config,   /* attribute data */
    },
    {
        HDLC_BLUETOOTH_MODULE_UART_TX_DATA_VALUE,              /* attribute handle */
        1,                                                     /* maxlen */
        1,                                                     /* curlen */
        app_bluetooth_module_uart_tx_data,                     /* attribute data */
    },
    {
        HDLC_BLUETOOTH_MODULE_UART_RX_DATA_VALUE,              /* attribute handle */
        244,                                                   /* maxlen */
        244,                                                   /* curlen */
        app_bluetooth_module_uart_rx_data,                     /* attribute data */
    },
    {
        HDLD_BLUETOOTH_MODULE_UART_RX_DATA_CLIENT_CHAR_CONFIG, /* attribute handle */
        2,                                                     /* maxlen */
        2,                                                     /* curlen */
        app_bluetooth_module_uart_rx_data_client_char_config,  /* attribute data */
    },
};

/* Number of Lookup Table entries */
const uint16_t app_gatt_db_ext_attr_tbl_size = 
    (sizeof(app_gatt_db_ext_attr_tbl) / sizeof(gatt_db_lookup_table_t));

/* Number of GATT initial value arrays entries */
const uint16_t app_gap_device_name_len = 9;
const uint16_t app_gap_appearance_len = (sizeof(app_gap_appearance));
const uint16_t app_gatt_service_changed_len = (sizeof(app_gatt_service_changed));
const uint16_t app_gatt_service_changed_client_char_config_len =
    (sizeof(app_gatt_service_changed_client_char_config));
const uint16_t app_dis_firmware_revision_string_len =
    (sizeof(app_dis_firmware_revision_string));
const uint16_t app_bluetooth_module_gpio_data_len = (sizeof(app_bluetooth_module_gpio_data));
const uint16_t app_bluetooth_module_command_len = (sizeof(app_bluetooth_module_command));
const uint16_t app_bluetooth_module_i2c_data_len = (sizeof(app_bluetooth_module_i2c_data));
const uint16_t app_bluetooth_module_i2c_rx_data_len =
    (sizeof(app_bluetooth_module_i2c_rx_data));
const uint16_t app_bluetooth_module_i2c_rx_data_client_char_config_len =
    (sizeof(app_bluetooth_module_i2c_rx_data_client_char_config));
const uint16_t app_bluetooth_module_uart_tx_data_len =
    (sizeof(app_bluetooth_module_uart_tx_data));
const uint16_t app_bluetooth_module_uart_rx_data_len =
    (sizeof(app_bluetooth_module_uart_rx_data));
const uint16_t app_bluetooth_module_uart_rx_data_client_char_config_len =
    (sizeof(app_bluetooth_module_uart_rx_data_client_char_config));
