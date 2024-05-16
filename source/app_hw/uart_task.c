/******************************************************************************
 * File Name: uart_task.c
 *
 * Description: This file contains the task that is used for thread-safe UART
 *              based debug.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright (2020), Cypress Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * (“Software”), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software (“EULA”).
 *
 * If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress’s integrated circuit products.
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
 * significant property damage, injury or death (“High Risk Product”). By
 * including Cypress’s product in a High Risk Product, the manufacturer of such
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

/* Header file includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "uart_task.h"
#include "app_bt_gatt_handler.h"

/* Queue length of message queue used for uart task */
//#define UART_QUEUE_SIZE 		(16u)

/* Maximum allowed length of a message */
//#define UART_MESSAGE_MAX_LEN	(100u)

/* Queue handle for uart message Queue */
//QueueHandle_t uartMessageQ;

#define CY_UART_TX	 P6_5
#define CY_UART_RX   P6_4

/* Buffer Size Definition */
#define UART_BUFFER_SIZE 244

// UART HAL object used by BSP for UART port
cyhal_uart_t cy_io_uart_obj;
cyhal_uart_cfg_t uart_config = {
	    .data_bits = 8,
	    .stop_bits = 1,
	    .parity = CYHAL_UART_PARITY_NONE,
		.rx_buffer = NULL,
		.rx_buffer_size = 0,
	};

uint8_t rx_data[UART_BUFFER_SIZE] = {'\0'};
size_t rx_length = 1;
uint8_t byte_received = 0;
volatile uint16_t byte_index = 0;
bool uart_clear_to_send_flag = true;

// Create a Queue to hold incoming UART bytes
QueueHandle_t xUartQueue;


/*******************************************************************************
* Function Name: void init_UART(void* pvParameters)
********************************************************************************
*
* Summary: This function initializes the Universal Asynchronous Receiver/Transmitter (UART)
*          with the specified parameters. It sets up the UART, configures it, creates a queue
*          for received bytes, registers a callback function for data reception, and enables
*          interruptions for data reception.
*
* Parameters:
*  void* pvParameters: Pointer to the parameters passed to the function. These parameters are
*                      not used in this function, indicating potential for future expansion or
*                      compatibility with a certain API.
*
* Return:
*  None
*/
void init_UART(void* pvParameters)
{
	cy_rslt_t result = cyhal_uart_init(&cy_io_uart_obj, CY_UART_TX, CY_UART_RX, NC, NC, NULL, &uart_config);
	if (result != CY_RSLT_SUCCESS) {
		handle_error(0);
	}

	result = cyhal_uart_configure(&cy_io_uart_obj, &uart_config);
	if (result != CY_RSLT_SUCCESS) {
		handle_error(0);
	}

	// Create the queue to hold received bytes. This can be moved to the main if you want.
	xUartQueue = xQueueCreate(UART_BUFFER_SIZE, sizeof(uint8_t));

	/* Register callback */
	cyhal_uart_register_callback(&cy_io_uart_obj, uart_rx_callback, NULL);

	/* Enable interruptions for data reception */
	cyhal_uart_enable_event(&cy_io_uart_obj, CYHAL_UART_IRQ_RX_NOT_EMPTY, CYHAL_ISR_PRIORITY_DEFAULT, true);

}

/*******************************************************************************
* Function Name: void uart_rx_callback(void *arg, cyhal_uart_event_t event)
********************************************************************************
*
* Summary: This function is a callback that gets triggered when a UART RX event occurs.
*          It checks if the UART RX buffer is not empty. If it's not, it reads a byte from
*          the UART, handles any errors, and sends the received byte to a FreeRTOS queue.
*
* Parameters:
*  void *arg: Pointer to the arguments passed to the function. These arguments are not used
*             in this function, indicating potential for future expansion or compatibility
*             with a certain API.
*  cyhal_uart_event_t event: The UART event that triggered the callback.
*
* Return:
*  None
*
*******************************************************************************/
void uart_rx_callback(void *arg, cyhal_uart_event_t event)
{
    cy_rslt_t result;
    if (event & CYHAL_UART_IRQ_RX_NOT_EMPTY) {
        result = cyhal_uart_read(&cy_io_uart_obj, &byte_received, &rx_length);
        if (result == CY_RSLT_SUCCESS) {
            handle_error(0);
        }
        // Instead of directly printing, send byte to the FreeRTOS Queue
        xQueueSendFromISR(xUartQueue, &byte_received, NULL);
    }
}

/*******************************************************************************
* Function Name: void clear_uart_buffer(void)
********************************************************************************
*
* Summary: This function clears the UART RX buffer.
*
* Parameters: None
*
* Return: None
*
*******************************************************************************/
void clear_uart_buffer()
{
	memset(rx_data, '\0', UART_BUFFER_SIZE);
}

/*******************************************************************************
* Function Name: void send_uart_message(void)
********************************************************************************
*
* Summary: This function checks whether it's safe to send a BT notification and, if so,
* calls the function to send the message, then resets indexes and buffers.
*
* Parameters: None
*
* Return: None
*
*******************************************************************************/
void send_uart_message()
{
	if (app_bluetooth_module_uart_rx_data_client_char_config[0]) {

//	   printf("UART RX: %s\n", rx_data);
	   app_bt_send_uart_message(rx_data, byte_index+1);

	}
	byte_index = 0;
	clear_uart_buffer();
}

/*******************************************************************************
* Function Name: void task_UART(void* pvParameters)
********************************************************************************
*
* Summary: This function is a task that continuously checks if there is data in the UART queue.
*          If there is, it receives the byte from the queue, prints it, and sends it to the
*          Bluetooth application.
*
* Parameters:
*  void* pvParameters: Pointer to the parameters passed to the function. These parameters are
*                      not used in this function, indicating potential for future expansion or
*                      compatibility with a certain API.
*
* Return:
*  None
*
*******************************************************************************/
void task_UART(void* pvParameters)
{
	uint8_t received_byte;
	while(true)
	{
		while(xQueuePeek(xUartQueue, &received_byte, 50))
		{
			if(xQueueReceive(xUartQueue, &received_byte, 50))
			{
			   // Process the received byte
			   rx_data[byte_index] = received_byte;
			   if(byte_index < UART_BUFFER_SIZE-1)
			   {
				   byte_index++;
			   } else {
				   // Buffer is full, send message immediately
//				   for (int i = 0; i < byte_index; i++){
//					   printf("%c", rx_data[i]);
//				   }
				   send_uart_message();
			   }
			}
		}
		if (byte_index > 0)
		{
//			for (int i = 0; i < byte_index; i++){
//								   printf("%c", rx_data[i]);
//							   }
		    send_uart_message();
		}
		//printf("%s", "Stuck in forever loop");
	 }
}

/*******************************************************************************
* Function Name: uint8_t* transmit_UART(uint8_t *uart_data, uint8_t len)
********************************************************************************
*
* Summary: This function transmits data over UART. It iterates over the uart_data array
*          and sends each byte using the cyhal_uart_putc function. If there is an error
*          during transmission, it prints an error message and breaks the loop.
*
* Parameters:
*  uint8_t *uart_data: Pointer to the data to be transmitted.
*  uint8_t len: The length of the data to be transmitted.
*
* Return:
*  NULL: This function does not return any value.
*
*******************************************************************************/
uint8_t* transmit_UART(uint8_t *uart_data, uint8_t len)
{
	cy_rslt_t result = cyhal_uart_putc(&cy_io_uart_obj, uart_data[0]);
	if (result != CY_RSLT_SUCCESS)
	{
		printf("UART send error: %lu\n", result);
	}

	return NULL;
}

/* [] END OF FILE */
