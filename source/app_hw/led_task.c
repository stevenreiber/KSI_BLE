/******************************************************************************
* File Name: led_task.c
*
* Description: This file contains FreeRTOS task that controls LED status.
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
#include "led_task.h"
#include "uart_task.h"
#include "cyhal.h"
#include "cybsp.h"
#include "task.h"

/*******************************************************************************
* Function Name: void task_LED(void *pvParameters)
********************************************************************************
*
* Summary: FreeRTOS task which controls the status of user LED1 and user LED2
*            User LED1 ON : BLE device connected with peer device
*            User LED1 OFF: BLE device disconnected with peer/ not connected to
*                           any peer device
*            User LED2 ON : GATT data being transmitted
*            User LED2 OFF: No GATT data transfer
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)
*
* Return:
*  None
*
*******************************************************************************/

void task_LED(void *pvParameters)
{
	 (void)pvParameters; // Just to avoid compiler warning about unused parameter.

	    /* Create an infinite loop. */
	    for(;;)
	    {
	        /* Toggle LED */
	    	cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED1,false); //This is used just to know that the LED pin is LOW (LED pin is used as GPIO pin in this project)
	    	cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED2,false); //This is used just to know that the LED pin is LOW (LED pin is used as GPIO pin in this project)
	        //cyhal_gpio_toggle((cyhal_gpio_t)CYBSP_USER_LED1);
	        //cyhal_gpio_toggle((cyhal_gpio_t)CYBSP_USER_LED2);

	        printf("LED Task \n");
	        /* Delay for a period of time. */
	        vTaskDelay(100);
	        /*Suspend task*/
	        vTaskSuspend(NULL);
	    }


//    /* Variable to store led status received from queue */
//    led_status_t ledStatus;
//
//    /* Variable used to store return values of RTOS APIs */
//    BaseType_t rtosApiResult;
//
//    /* Remove warning for unused variable */
//    (void)pvParameters;
//
//    /* Repeatedly running part of the task */
//    for(;;)
//    {
//        /* Block until a command has been received over led_cmdQ */
//        rtosApiResult = xQueueReceive(led_cmdQ, &ledStatus, portMAX_DELAY);
//
//        /* Command has been received from led_cmdQ */
//        if(rtosApiResult == pdTRUE)
//        {
//            /* Turn user LED1 ON or OFF based on BLE connection status */
//            if(ledStatus.conn_led == CYBSP_LED_STATE_ON)
//            {
//                cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED1,
//                                  CYBSP_LED_STATE_ON);
//            }
//            else
//            {
//                cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED1,
//                                  CYBSP_LED_STATE_OFF);
//            }
//
//            /* Turn user LED2 ON or OFF based on GATT data traffic */
//            if(ledStatus.data_led == CYBSP_LED_STATE_ON)
//            {
//                cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED2,
//                                  CYBSP_LED_STATE_ON);
//            }
//            else
//            {
//                cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED2,
//                                  CYBSP_LED_STATE_OFF);
//            }
//        }
//    }
}
/* [] END OF FILE */

