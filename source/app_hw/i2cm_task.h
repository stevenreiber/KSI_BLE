/*
 * i2cm_task.h
 *
 *  Created on: Jun 26, 2023
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

#ifndef SOURCE_APP_HW_I2CM_TASK_H_
#define SOURCE_APP_HW_I2CM_TASK_H_

/*******************************************************************************
 * Header Files
 ******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cyhal_i2c.h"
#include "command.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* Delay of 1000ms between commands */
#define CMD_TO_CMD_DELAY        (1000UL)

/* Packet positions */
#define PACKET_SOP_POS          (0UL)
#define PACKET_CMD_POS          (1UL)
#define PACKET_EOP_POS          (2UL)

/* Start and end of packet markers */
#define PACKET_SOP              (0x01UL)
#define PACKET_EOP              (0x17UL)

/* I2C slave address to communicate with */
#define I2C_SLAVE_ADDR          (0x24UL)

/* I2C bus frequency */
#define I2C_FREQ                (400000UL)

/* Command valid status */
#define STATUS_CMD_DONE         (0x00UL)

/* Packet size */
#define PACKET_SIZE             (3UL)

/* Max number of iterations */
#define MAX_NUM_ITERATIONS    10

/* Buffer size */
#define BUFFER_SIZE 207


/* Global buffer */
extern uint8_t i2c_hex_char_buffer[BUFFER_SIZE]; //After sending al the I2C frame, the buffer has to be empty or clean
extern uint8_t i2c_read_buffer[BUFFER_SIZE-7]; // Buffer para leer datos
extern bool i2c_flag;

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void init_I2Cm(void* pvParameters);
bool isSingleBitSet(uint8_t x);
void i2c_send_data(uint8_t *data);
uint8_t* i2c_read_data(uint8_t *data);

#endif /* SOURCE_APP_HW_I2CM_TASK_H_ */
