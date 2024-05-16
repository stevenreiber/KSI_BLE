/*******************************************************************************
 * THIS SAMPLE CODE IS PROVIDED “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL ARROW ELECTRONICS, INC.
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *OR BUSINESS INTERRUPTION) SUSTAINED BY YOU OR A THIRD PARTY, HOWEVER CAUSED
 *AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * ON ARISING IN ANY WAY OUT OF THE USE OF THIS SAMPLE CODE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

/*******************************************************************************
 * Header Files
 *******************************************************************************/
#include <FreeRTOS.h>
#include <task.h>

#include "app_bt_bonding.h"
#include "app_bt_event_handler.h"
#include "app_bt_gatt_handler.h"
#include "app_bt_utils.h"
#include "app_flash_common.h"
#include "app_hw_device.h"
#include "command.h"
#include "cy_retarget_io.h"
#include "cybsp.h"
#include "cybsp_bt_config.h"
#include "cybt_platform_config.h"
#include "cycfg_bt_settings.h"
#include "gpio_task.h"
#include "i2cm_task.h"
#include "led_task.h"
#include "mtb_kvstore.h"
#include "uart_task.h"
#include "wiced_bt_stack.h"
#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

/******************************************************************************
 * Macros
 ******************************************************************************/

/* Priority of user tasks in this application. Valid range of task priority is
 * 0 to ( configMAX_PRIORITIES - 1 ), where configMAX_PRIORITIES is defined
 * within FreeRTOSConfig.h.
 */
#define BLE_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define UART_TASK_PRIORITY (configMAX_PRIORITIES - 4)
#define I2CM_TASK_PRIORITY (configMAX_PRIORITIES - 4)

#define BLE_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)
#define UART_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 2)
#define I2CM_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)

/* FreeRTOS task, queue and timer handles for this application*/
TaskHandle_t ble_task_handle, uart_task_handle,
    i2cm_task_handle;  // i2cm_write_task_handle;

/******************************************************************************
 * Function Definitions
 ******************************************************************************/

/**
 * Function Name : main
 *
 * Function Description :
 *   @brief Entry point to the application. Set device configuration and start
 *   BT stack initialization.  The actual application initialization will happen
 *   when stack reports that BT device is ready.
 *
 *   @param: None
 *
 *   @return: None
 */
int main() {
    cy_rslt_t cy_result;
    wiced_result_t wiced_result;
    BaseType_t rtos_api_result_uart = pdPASS;
    BaseType_t rtos_api_result_i2cm = pdPASS;
    // BaseType_t rtos_api_result_i2cm_write = pdPASS;

    /* Initialize the board support package */
    cy_result = cybsp_init();

    if (CY_RSLT_SUCCESS != cy_result) {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);

    printf(
        "\r\n****************************************\n"
        "Karl Storz Imaging BLE Application Start\n"
        "****************************************\n");
    printf("Firmware revision: %s\n", app_dis_firmware_revision_string);

    /* Initialize the FPGA UART */
    init_UART(NULL);

    /* Initialize the GPIOs */
    init_GPIO(NULL);

    /* Initialize the I2C */
    init_I2Cm(NULL);

    /* Configure platform specific settings for the BT device */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /*Initialize the block device used by kv-store for performing
     * read/write operations to the flash*/
    app_kvstore_bd_config(&block_device);

    /* Register call back and configuration with stack */
    wiced_result =
        wiced_bt_stack_init(app_bt_management_callback, &wiced_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if (WICED_BT_SUCCESS == wiced_result) {
        printf("Bluetooth Stack Initialization Successful\n");
    } else {
        printf("Bluetooth Stack Initialization failed\n");
        CY_ASSERT(0);
    }

    /*UART initialization*/
    rtos_api_result_uart |=
        xTaskCreate(task_UART, "UART Task", UART_TASK_STACK_SIZE, NULL,
                    UART_TASK_PRIORITY, &uart_task_handle);

    if (pdPASS == rtos_api_result_uart &&
        rtos_api_result_i2cm)  // && rtos_api_result_i2cm_write)
    {
        printf("Starting scheduler\r\n");
        /* Start the RTOS scheduler. This function should never return */
        vTaskStartScheduler();

        /* Program should never reach here! */
        printf("[Error] : FreeRTOS scheduler failed to start\r\n");
    } else {
        printf("[Error] : FreeRTOS failed to create task\r\n");
    }

    /* Should never get here */
    CY_ASSERT(0);

    for (;;) {
    }
}
