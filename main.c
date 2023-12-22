/******************************************************************************
 * File Name:   main.c
 *
 * Description: This project demonstrates the implementation of multi-peripheral
 *              functionality using AIROC&trade; CYW20829, PSoC6 Bluetooth LE device,
 *              and ModusToolboxsoftware environment.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cycfg_gap.h"
#include "cycfg_gatt_db.h"
#include "cycfg_bt_settings.h"
#include "cybsp_bt_config.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_gatt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "temperature_task.h"
#include "rgb_led_task.h"
#include "app_bt_task.h"
#include <stdlib.h>

/*******************************************************************************
 * Macros
 ********************************************************************************/

/* Priorities of user tasks in this project. configMAX_PRIORITIES is defined in
 * the FreeRTOSConfig.h and higher priority numbers denote high priority tasks.
 */
#define TASK_TEMPERATURE_PRIORITY      (configMAX_PRIORITIES-3u)
#define TASK_RGB_LED_PRIORITY          (configMAX_PRIORITIES-2u)
#define TASK_BT_PRIORITY               (configMAX_PRIORITIES-1u)

/* Stack sizes of user tasks in this project */
#define TASK_TEMPERATURE_STACK_SIZE    (configMINIMAL_STACK_SIZE)
#define TASK_RGB_LED_STACK_SIZE        (2u*configMINIMAL_STACK_SIZE)
#define TASK_BT_STACK_SIZE            (1024u)

/* Queue lengths of message queues used in this project */
#define SINGLE_ELEMENT_QUEUE           (1u)

/******************************************************************************
 * Global Variables
 ******************************************************************************/
/* This enables RTOS aware debugging. */
volatile int uxTopUsedPriority;

/******************************************************************************
 * Function Definitions
 ******************************************************************************/

/*******************************************************************************
 * Function Name : main
 * *****************************************************************************
 * Summary :
 *   Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 *
 * Parameters:
 *    None
 *
 * Return:
 *    int
 ******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* This enables RTOS aware debugging in OpenOCD. */
    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    /* Initialize the board support package */
    result = cybsp_init();
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0u);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init( CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
            CY_RETARGET_IO_BAUDRATE);
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0u);
    }
    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("******************** BTSTACK FreeRTOS Example ************************\n");
    printf("****************** Bluetooth LE Multi-Peripheral *****************************\n");

    wiced_result_t wiced_result = WICED_BT_SUCCESS;

    /* Configure platform specific settings for the BT device */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /* Register call back and configuration with stack */
    wiced_result = wiced_bt_stack_init(app_bt_management_cb, &wiced_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if( CY_RSLT_SUCCESS != wiced_result)
    {
        printf("Bluetooth stack initialization failed!\r\n");
        CY_ASSERT(0);
    }

    /**
     * Create the queues.bt_l2cap_update_app_bt_conn_params
     * See the respective data-types for details of queue contents.
     */
    rgb_led_command_data_q  = xQueueCreate(SINGLE_ELEMENT_QUEUE,
            sizeof(rgb_led_command_data_t));
    if(NULL == rgb_led_command_data_q)
    {
        printf("Failed to create the queue!\r\n");
        CY_ASSERT(0u);
    }

    temperature_command_q  = xQueueCreate(SINGLE_ELEMENT_QUEUE,
            sizeof(temperature_command_t));
    if(NULL == temperature_command_q)
    {
        printf("Failed to create the queue!\r\n");
        CY_ASSERT(0u);
    }

    app_bt_command_data_q  = xQueueCreate(SINGLE_ELEMENT_QUEUE,
            sizeof(app_bt_command_data_t));
    if(NULL == app_bt_command_data_q)
    {
        printf("Failed to create the queue!\r\n");
        CY_ASSERT(0u);
    }

    /* Create the user tasks. See the respective task definition for more
     * details of these tasks.
     */
    if (pdPASS != xTaskCreate(task_temperature, "temperature Task",
            TASK_TEMPERATURE_STACK_SIZE,
            NULL, TASK_TEMPERATURE_PRIORITY, NULL))
    {
        printf("Failed to create the temperature task!\r\n");
        CY_ASSERT(0u);
    }

    if (pdPASS != xTaskCreate(task_rgb_led, "rgb Led Task", TASK_RGB_LED_STACK_SIZE,
            NULL, TASK_RGB_LED_PRIORITY, NULL))
    {
        printf("Failed to create the RGB LED task!\r\n");
        CY_ASSERT(0u);
    }

    if (pdPASS != xTaskCreate(task_bluetooth, "Bluetooth Task", TASK_BT_STACK_SIZE,
            NULL, TASK_BT_PRIORITY, NULL))
    {
        printf("Failed to create the BLE task!\r\n");
        CY_ASSERT(0u);
    }

    /* Start the RTOS scheduler. This function should never return */
    vTaskStartScheduler();

    /*~~~~~~~~~~~~~~~~~~~~~ Should never get here! ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    /* RTOS scheduler exited */
    /* Halt the CPU if scheduler exits */
    CY_ASSERT(0u);
}

/* [] END OF FILE */
