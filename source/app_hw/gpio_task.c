/******************************************************************************
 * File Name: gpio_task.c
 *
 * Description: This file contains FreeRTOS task that controls GPIO status.
 *
 * Related Document: README.md
 *
 *******************************************************************************
 * Copyright (2020), Cypress Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * ("Software"), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries ("Cypress") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 *******************************************************************************/
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

/******************************************************************************
 * Include header files
 *****************************************************************************/
#include "gpio_task.h"

const int gpio_list[25] = {
    P0_4,
    P0_5,
    P5_2,
    P5_3,
    P5_4,
    P5_5,
    P5_6,
    P6_2,
    P6_3,
    P7_1,
    P7_2,
    P7_7,
    P9_3,
    P9_4,
    P9_5,
    P9_6,
    P10_0,
    P10_1,
    P10_2,
    P10_3,
    P10_4,
    P10_5,
    P10_6,
    P12_6,
    P12_7
};

/*******************************************************************************
 * Function Name: void init_GPIO(void* pvParameters)
 ********************************************************************************
 *
 * Summary: This function initializes the General Purpose Input/Output (GPIO) pins
 *          specified in the gpio_list array.
 *
 * Parameters:
 *  void* pvParameters: Pointer to the parameters passed to the function, which are not
 *                      used in this function (indicating potential for future expansion
 *                      or compatibility with a certain API).
 *
 * Return:
 *  None
 *
 *******************************************************************************/

void init_GPIO(void * pvParameters) {
    // Set all GPIO drive states to input pull-down, default 0
    for (uint8_t i = 0; i < sizeof(gpio_list) / sizeof(gpio_list[0]); i++) {
        cyhal_gpio_init((cyhal_gpio_t) gpio_list[i], CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, 0U);
    }

    // Set FPGA pin drive states to input as a High-Z
    cyhal_gpio_free(PS_POR_B);
    cyhal_gpio_init(PS_POR_B, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, LOW);

    // Set Switch pin to output configured as a pull-down
    cyhal_gpio_init(SWITCH_RESET_B, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULLDOWN, HIGH);
}

/*******************************************************************************
* Function Name: uint8_t* send_GPIO_values(void* pvParameters)
********************************************************************************
*
* Summary: This function reads the current states of the GPIOs listed in the
*          gpio_list array, packs these states into a dynamically allocated buffer,
*          and returns a pointer to this buffer.
*

* Parameters:
*  void* pvParameters: Pointer to the parameters passed to the function.
* Return:
*  uint8_t*: A pointer to the dynamically allocated buffer containing the packed
*            GPIO states.
*
*******************************************************************************/

uint8_t * send_GPIO_values(void * pvParameters) {
    /* Get GPIO states and pack buffer*/
    uint32_t gpio_states = 0;
    /*  Dynamically allocate memory for buffer*/
    uint8_t * buffer = (uint8_t * ) malloc(4 * sizeof(uint8_t));

    if (buffer == NULL) {
        // Handle memory allocation failure
        return NULL;
    }

    for (uint8_t j = 0; j < sizeof(gpio_list) / sizeof(gpio_list[0]); j++) {
        if (cyhal_gpio_read(gpio_list[j])) {
            gpio_states |= (1 << j);
        }
    }

    // See how many bytes do I need to send to the buffer
    buffer[0] = gpio_states & 0xFF;
    buffer[1] = (gpio_states >> 8) & 0xFF;
    buffer[2] = (gpio_states >> 16) & 0xFF;
    buffer[3] = gpio_states >> 24;

    printf("GPIO States: 0x%02X%02X%02X%02X\n", buffer[3], buffer[2], buffer[1], buffer[0]);

    return buffer;
}

/*******************************************************************************
* Function Name: void reset_fpga(void)
********************************************************************************
*
* Summary: This function performs a hardware reset of the FPGA by manipulating
*          the FPGA power-on reset (POR) pin, designated as PS_POR_B.
*
* Parameters:
*  None
*
* Return:
*  None

*******************************************************************************/

void reset_fpga(void) {
    //	//drive P0.4 low for about 10 ms, then undrive it (leave it high-Z)
    cyhal_gpio_free(PS_POR_B);
    cyhal_gpio_init(PS_POR_B, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, LOW);
    cyhal_gpio_write(PS_POR_B, LOW);
    cyhal_system_delay_ms(DELAY_10MS);
    cyhal_gpio_free(PS_POR_B);
    cyhal_gpio_init(PS_POR_B, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, LOW);

}

/*******************************************************************************
* Function Name: void reset_switch(void)
********************************************************************************
*
* Summary: This function performs a reset of a switch by toggling the associated
*          reset control pin, designated as SWITCH_RESET_B.

*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/

void reset_switch(void) {
    //drive P9.2 low for about 10 ms, then drive it high (leave it high)
    cyhal_gpio_write(SWITCH_RESET_B, LOW);
    cyhal_system_delay_ms(DELAY_10MS);
    cyhal_gpio_write(SWITCH_RESET_B, HIGH);

}
