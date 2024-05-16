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
#include "command.h"

/* FreeRTOS task handles for this application*/
//extern TaskHandle_t ble_task_handle, uart_task_handle, i2cm_read_task_handle, i2cm_write_task_handle;

/*******************************************************************************
 * Function Name: void handle_error(uint32_t status)
 ********************************************************************************
 *
 * Summary: This function checks if the provided status indicates an error and
 *          asserts if the status is not CY_RSLT_SUCCESS.
 *
 * Parameters:
 *  uint32_t status: Status code to be checked.
 * Return:
 *  None
 *
 *******************************************************************************/
void handle_error(uint32_t status) {
    if (status != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }
}

/*******************************************************************************
 * Function Name: void check_command(uint8_t command)
 ********************************************************************************
 *
 * Summary: This function takes a command byte and executes the corresponding
 *          operation. Each command represents a different task or action to be
 *          performed, such as reading GPIO status, interacting with I2C devices,
 *          handling UART communication, or managing system resets through GPIO pins.
 *          The function uses a switch-case statement to determine which operation
 *          corresponds to the given command and resumes the appropriate task that
 *          is responsible for executing the specified operation.
 *
 * Parameters:
 *  uint8_t command: The command byte that determines which operation should be
 *                   executed.
 *
 * Return:
 *  None
 *
 *******************************************************************************/

void check_command(uint8_t command) {
    switch (command) {
        /* Triggers a reset for an FPGA using a specific GPIO pin. */
		case SET_PIN_RESET_FPGA:
			// Insert the logic for "P0.4 pin sets (FPGA reset)" here
			printf("Executing FPGA_RESET\n");
			reset_fpga();
			break;

        /* Triggers a system reset through a dedicated reset switch pin. */
		case SET_PIN_RESET_SWITCH:
			// Insert the logic for "Sets pin P9.2 (reset switch)" here
			printf("Executing I2C_SWITCH_RESET\n");
			reset_switch();
			break;

		default:
			printf("Unknown command!\n");
	}
}

/*******************************************************************************
 * Function Name: void suspend_tasks(TaskHandle_t xTaskToSuspend)
 ********************************************************************************
 *
 * Summary: This function suspends a FreeRTOS task if the provided task handle
 *          is not NULL. Suspending a task prevents it from being scheduled to run
 *          until it has been explicitly resumed, allowing for controlled execution
 *          of concurrent tasks in a multitasking environment.
 *
 * Parameters:
 *  TaskHandle_t xTaskToSuspend: The handle of the task to be suspended.
 *
 * Return:
 *  None
 *
 *******************************************************************************/

void suspend_tasks(TaskHandle_t xTaskToSuspend) {
    if (xTaskToSuspend != NULL) {
        vTaskSuspend(xTaskToSuspend);
    }
}
