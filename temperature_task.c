/*******************************************************************************
 * File Name: temperature_task.c
 *
 * Description: This file contains the task that handles temperature data.
 *
 * Related Document: README.md
 *
 ********************************************************************************
 * (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
 ********************************************************************************
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
 *****************************************​**************************************/

/*******************************************************************************
 * Header files includes
 *******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "rgb_led_task.h"
#include "app_bt_task.h"
#include "temperature_task.h"
#include "cy_retarget_io.h"
#include "math.h"

/*******************************************************************************
 * Macro Definitions
 *******************************************************************************/
#define STATUS_IDLE_INTERVAL   (portMAX_DELAY)

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
static void adc_init(void);
static void temperature_timer_callback(TimerHandle_t xTimer);

/*******************************************************************************
 * Global variables
 *******************************************************************************/
QueueHandle_t temperature_command_q;

cyhal_adc_t adc;

cyhal_adc_channel_t adc_chan_0_obj;

static const cyhal_adc_channel_config_t DEFAULT_CHAN_CONFIG =
{ .enable_averaging = false, .min_acquisition_ns = 10u, .enabled = true };
TimerHandle_t scan_timer_handle;

/*******************************************************************************
 * Function Name: task_temperature
 ********************************************************************************
 * Summary:
 *  Task that reads temperature data from thermistor circuit
 *
 * Parameters:
 *  void *param : Task parameter defined during task creation (unused)
 *
 *******************************************************************************/
void task_temperature(void* param)
{
    BaseType_t rtos_api_result = pdFAIL;
    temperature_command_t temperature_cmd;

    /* Remove warning for unused parameter */
    (void)param;

    /* Initialize timer */
    scan_timer_handle = xTimerCreate (" Timer", STATUS_IDLE_INTERVAL,
            pdTRUE, NULL, temperature_timer_callback);

    if(NULL == scan_timer_handle)
    {
        printf("Timer initilization failed!\r\n");
        CY_ASSERT(0u);
    }

    /* ADC initialization */
    adc_init();

    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a Temperature command has been received over queue */
        rtos_api_result = xQueueReceive(temperature_command_q, &temperature_cmd,
                portMAX_DELAY);

        /* Command has been received from temperature_cmd */
        if(pdTRUE == rtos_api_result)
        {
            switch(temperature_cmd)
            {
            case TEMPERATURE_STOP:
            {
                xTimerStop(scan_timer_handle, 0u);

                break;
            }

            case TEMPERATURE_START:
            {
                xTimerStart(scan_timer_handle, 0u);
                xTimerChangePeriod(scan_timer_handle, 1000, 0u);

                break;
            }

            case TEMPERATURE_PROCESS:
            {
                #if !defined(CYW20829B0LKML) && !defined(CYW20829B1010)
                /* Measure voltage drop on the reference resistor */
                cyhal_gpio_write(THERM_VDD, 0u);
                cyhal_gpio_write(THERM_GND, 1u);
                uint16_t voltage_ref = cyhal_adc_read_u16(&adc_chan_0_obj);

                /* Measure voltage drop on the thermistor */
                cyhal_gpio_write(THERM_VDD, 1u);
                cyhal_gpio_write(THERM_GND, 0u);
                uint16_t voltage_therm = cyhal_adc_read_u16(&adc_chan_0_obj);

                /* Disable power to thermistor circuit */
                cyhal_gpio_write(THERM_VDD, 0u);

                /* Calculate thermistor resistance */
                float rThermistor =((R_REFERENCE * (voltage_therm)) / ((float)(voltage_ref)));

                /* Calculate thermistor temperature */
                float temperature = (B_CONSTANT / (logf(rThermistor / R_INFINITY))) +
                        ABSOLUTE_ZERO;
                #else
                uint16_t voltage_therm = cyhal_adc_read_u16(&adc_chan_0_obj);
                /* Calculate thermistor resistance */
                float rThermistor =((R_REFERENCE * (voltage_therm)) / ((float)(ADC_MAX - voltage_therm)));
                /* Calculate thermistor temperature */
                float temperature = (B_CONSTANT / (logf(rThermistor / R_INFINITY))) +
                        ABSOLUTE_ZERO;
                #endif
                app_bt_command_data_t app_bt_command = { .command = SEND_TEMP_INDICATION,
                        .data = temperature };

                xQueueSendToBack(app_bt_command_data_q, &app_bt_command, 0u);

                break;
            }

            /* Invalid command */

            default:
            {
                break;
            }
            }
        }
    }
}

/*******************************************************************************
 * Function Name: adc_init
 ********************************************************************************
 * Summary:
 *  This function starts adc.
 *
 *******************************************************************************/
static void adc_init(void)
{
    cy_rslt_t result;

#if !defined(CYW20829B0LKML) && !defined(CYW20829B1010)
    result = cyhal_gpio_init(THERM_VDD, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0u);
    if (CY_RSLT_SUCCESS == result)
    {
        cyhal_gpio_init(THERM_GND, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0u);
    }
#endif
    /* Initialize adc */
    result = cyhal_adc_init(&adc, THERM_OUT1, NULL);

    if(CY_RSLT_SUCCESS != result)
    {
        printf("ADC Initialization failed!\r\n");
    }

    cyhal_adc_channel_init_diff(&adc_chan_0_obj, &adc, THERM_OUT1, CYHAL_ADC_VNEG,
            &DEFAULT_CHAN_CONFIG);
}

/*******************************************************************************
 * Function Name: temperature_timer_callback
 ********************************************************************************
 * Summary:
 *  Temperature timer callback. This function sends a command to start Temperature.
 *
 * Parameters:
 *  TimerHandle_t xTimer (unused)
 *
 *******************************************************************************/
static void temperature_timer_callback(TimerHandle_t xTimer)
{
    temperature_command_t command = TEMPERATURE_PROCESS;

    (void)xTimer;

    /* Send command to start CapSense scan */
    xQueueSend(temperature_command_q, &command, 0u);

}

/* END OF FILE [] */
