/*
 * i2cm_task.c
 *
 *  Created on: Jun 26, 2023
 */
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

/* Includes */
#include "i2cm_task.h"

/* I2C Pins Definition */
#define CY_I2C_SCL P9_0
#define CY_I2C_SDA P9_1

/* Data Indices */
#define DATA_N_INDEX 5
#define DATA_START_INDEX 7

/* Global Variables */
cyhal_i2c_t mI2C;
cyhal_i2c_cfg_t mI2C_cfg;

/* Current I2C switch address and bus */
uint8_t current_switch_bus = 0xFF;

/* Write I2C data variables */
//uint8_t i2c_buffer_index = 0;
uint8_t data_n = 0;
bool i2c_flag = true;

/* Read I2C data vairables */
//uint16_t i2c_read_index = 0;

/*******************************************************************************
 * Function Name: void task_I2Cm(void* pvParameters)
 ********************************************************************************
 * Summary:
 *  Task that sends data through I2C.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void init_I2Cm(void * pvParameters) {
    cy_rslt_t result;

    /* I2C Master configuration settings */
//    printf("Configuring I2C Master...\n");
    mI2C_cfg.is_slave = false;
    mI2C_cfg.address = 0;
    mI2C_cfg.frequencyhal_hz = I2C_FREQ;

    /* Init I2C master */
    result = cyhal_i2c_init( & mI2C, CY_I2C_SDA, CY_I2C_SCL, NULL);
    /* I2C master init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS) {
        handle_error(0);
    }

    /* Configure I2C Master */
    result = cyhal_i2c_configure( & mI2C, & mI2C_cfg);
    /* I2C master configuration failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS) {
        handle_error(0);
    }

    /* Enable interrupts */
    __enable_irq();
}

/*******************************************************************************
 * Function Name: bool isSingleBitSet(uint8_t x)
 ********************************************************************************
 * Summary:
 *  Function that verifies whether only a single bit in a byte is set.
 *
 * Parameters:
 *  uint8_t x : The byte to check
 *
 * Return:
 *  bool : True if only single bit is set, false otherwise
 *******************************************************************************/
bool isSingleBitSet(uint8_t x) {
    return (x != 0) && ((x & (x - 1)) == 0);
}

/*******************************************************************************
 * Function Name: void i2c_send_data(uint8_t *data, uint8_t len, uint8_t data_n)
 ********************************************************************************
 * Summary:
 *  Function that sends data via I2C.
 *
 * Parameters:
 *  uint8_t *data :  Pointer to the data to be sent
 *  uint8_t len :    Length of the data to be sent
 *  uint8_t data_n : Number of data to be sent
 *
 * Return:
 *  void
 *
 * Note:
 *  This function prepares and sends two frames of data via I2C. (Frame 1, if applicable, and Frame 2)
 *  If the buffer is full, it changes the ADDR of the slave to a control register definition,
 *  prepares the data for Frame 1 and Frame 2, and sends them.
 *  After sending the data, it resets the the buffer index.
 *******************************************************************************/
void i2c_send_data(uint8_t * data) {

    cy_rslt_t result;
    uint8_t switch_address = data[0];
    uint8_t switch_bus = data[1];
    uint8_t device_address = data[2];
    uint8_t data_n = data[5];
    uint8_t frame2_data[data_n];

    if (isSingleBitSet(switch_bus)) {
        // Get bit position corresponding to integer bus number
        uint8_t i = 1, bus_number = 0;
        while (!(i & switch_bus)) {
            i = i << 1;
            ++bus_number;
        }

        // Prepare data for Frame 1 (I2C Switch bus selection)
        if (switch_bus != current_switch_bus) {
            current_switch_bus = switch_bus;
            result = cyhal_i2c_master_write( & mI2C, switch_address, & switch_bus, 1, 1000, true);
            if (result != CY_RSLT_SUCCESS) {
                handle_error(0);
            } else {
                printf("I2C Switch set to bus: %i\n", bus_number);
            }
        }

        // Prepare data for Frame 2 (Data)
        // Pack data
        for (int i = 0; i < data_n; i++) {
            frame2_data[i] = data[DATA_START_INDEX + i];
        }
        // Send data
        result = cyhal_i2c_master_write( & mI2C, device_address, frame2_data, data_n, 1000, true);
        if (result != CY_RSLT_SUCCESS) {
            printf("Error writing data to device 0x%x on bus %i\n", device_address, bus_number);
        }
    } else {
        printf("Error on channel bit. Send BLE characteristic data again");
    }

    // Clear buffer after sending the I2C frame
    memset(i2c_hex_char_buffer, 0x00, BUFFER_SIZE);
}

/*******************************************************************************
 * Function Name: uint8_t* i2c_read_data(void* pvParameters)
 ********************************************************************************
 * Summary:
 *  Function that receives data via I2C.
 *
 * Parameters:
 *  void* pvParameters :  Pointer to the data to be sent
 *
 * Return:
 *  Pointer to i2c_read_buffer containing data read
 *
 * Note:
 *  This function prepares and sends two frames of data via I2C. (Frame 1, if applicable, and Frame 2)
 *  Handles "register-less" reads, as well as single/dual register byte reads.
 *******************************************************************************/
uint8_t * i2c_read_data(uint8_t * data) {

    cy_rslt_t result;
    uint8_t switch_address = data[0];
    uint8_t switch_bus = data[1];
    uint8_t device_address = data[2];
    uint8_t data_n = data[5];

    if (isSingleBitSet(switch_bus)) {
        // Prepare data for Frame 1 (I2C Switch bus selection)
        if (switch_bus != current_switch_bus) {
            current_switch_bus = switch_bus;
            result = cyhal_i2c_master_write( & mI2C, switch_address, & switch_bus, 1, 1000, true);
            if (result != CY_RSLT_SUCCESS) {
                handle_error(0);
            }
        }
        // If register high byte is 0 but low byte is not (Standard register read)
        if ((data[3] == 0) && (data[4] != 0)) {
            uint8_t register_low = i2c_hex_char_buffer[4];
            // Send the low byte
            result = cyhal_i2c_master_write( & mI2C, device_address, & register_low, 1, 1000, false);
            if (result != CY_RSLT_SUCCESS) {
                handle_error(0);
            }
        }
        // Both register high and register low bytes are non-0 (Two byte register read)
        else if ((data[3] != 0) && (data[4] != 0)) {
            uint8_t register_high_low[2] = {
                data[3],
                data[4]
            };
            // Send the low byte
            result = cyhal_i2c_master_write( & mI2C, device_address, register_high_low, 2, 1000, false);
            if (result != CY_RSLT_SUCCESS) {
                handle_error(0);
            }
        }
        // If register high and register low bytes are 0 (Register-less read)
        // Write nothing

        // Execute read
        result = cyhal_i2c_master_read( & mI2C, device_address, i2c_read_buffer, data_n, 1000, true);
        if (result != CY_RSLT_SUCCESS) {
            handle_error(0);
        }

        // Print results
        printf("I2C data received:");
        for (uint8_t i = 0; i < data_n; i++) {
            printf(" %02X", i2c_read_buffer[i]);
        }
        printf("\n");
    } else {
        printf("Bus Bit Error");
    }

    return i2c_read_buffer;
}
