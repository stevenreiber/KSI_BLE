/*
 * command.h
 *
 *  Created on: Sep 25, 2023
 *      Author: 154674
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

#ifndef SOURCE_APP_HW_COMMAND_H_
#define SOURCE_APP_HW_COMMAND_H_

/*******************************************************************************
 * Header Files
 ******************************************************************************/
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>
#include <inttypes.h>
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cyhal.h"
#include "cyhal_gpio.h"
#include "gpio_task.h"

/*******************************************************************************
 * Macros Definitions
 ******************************************************************************/
#define SET_PIN_RESET_FPGA            0xD2 // Sets pin P0.4 (reset FPGA)
#define SET_PIN_RESET_SWITCH          0xFC // Sets pin P9.2 (reset switch)

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void handle_error(uint32_t status);
void check_command(uint8_t command);
void suspend_tasks(TaskHandle_t xTaskToSuspend);

#endif /* SOURCE_APP_HW_COMMAND_H_ */
