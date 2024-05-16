/*
 * gpio_task.h
 *
 *  Created on: Sep 7, 2023
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

#ifndef SOURCE_APP_HW_GPIO_TASK_H_
#define SOURCE_APP_HW_GPIO_TASK_H_

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

/*******************************************************************************
 * Macros Definitions
 ******************************************************************************/
#define PS_POR_B             P0_4
#define SWITCH_RESET_B       P9_2
#define HIGH                 1
#define LOW                  0
#define DELAY_10MS           10UL

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void init_GPIO(void* pvParameters);
void task_GPIO(void* pvParameters);
void reset_fpga(void);
void reset_switch(void);
uint8_t* send_GPIO_values(void* pvParameters);



#endif /* SOURCE_APP_HW_GPIO_TASK_H_ */
