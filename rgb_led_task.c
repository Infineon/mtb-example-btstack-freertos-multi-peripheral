/******************************************************************************
 * File Name: rgb_led_task.c
 *
 * Description: This file contains the task that handles RGB led.
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
 * Header file includes
 ******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cy_retarget_io.h"
#include "rgb_led_task.h"

/*******************************************************************************
 * Macro Definitions
 *******************************************************************************/
#define PWM_LED_FREQ_HZ     (1000000u)  /* in Hz */

/* subtracting from 100 since the LED is connected in active low configuration */
#define GET_DUTY_CYCLE(x)   (100 - x)

/*******************************************************************************
 * Global variable
 ******************************************************************************/
/* Queue handle used for LED data */
QueueHandle_t rgb_led_command_data_q;

/*******************************************************************************
 * Function Name: task_rgb_led
 ********************************************************************************
 * Summary:
 *  Task that controls the color and intensity of the RGB LED
 *
 * Parameters:
 *  void *param : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void task_rgb_led(void* param)
{
    cyhal_pwm_t pwm_red;
    cyhal_pwm_t pwm_blue;
    cyhal_pwm_t pwm_green;

    BaseType_t rtos_api_result = pdFAIL;
    rgb_led_command_data_t rgb_led_cmd_data;
    cy_rslt_t cy_result = CY_RSLT_SUCCESS;

    /* Suppress warning for unused parameter */
    (void)param;

    /* Configure the TCPWM for driving RED led */
    cy_result = cyhal_pwm_init_adv(&pwm_red, CYBSP_LED_RGB_RED , NC, CYHAL_PWM_RIGHT_ALIGN, true, 0u, false, NULL);
    if (CY_RSLT_SUCCESS != cy_result  )
    {
        printf("PWM initialization failed!\r\n");
    }

    if(CY_RSLT_SUCCESS != cyhal_pwm_set_duty_cycle(&pwm_red, 100,
            PWM_LED_FREQ_HZ))
    {
        printf("PWM failed to set dutycycle!");
    }

    if(CY_RSLT_SUCCESS != cyhal_pwm_start(&pwm_red))
    {
        printf("PWM failed to start!");
    }

    /* Configure the TCPWM for driving GREEN led */
    cy_result = cyhal_pwm_init_adv(&pwm_green, CYBSP_LED_RGB_GREEN , NC, CYHAL_PWM_RIGHT_ALIGN, true, 0u, false, NULL);
    if (CY_RSLT_SUCCESS != cy_result  )
    {
        printf("PWM initialization failed!\r\n");
    }

    if(CY_RSLT_SUCCESS != cyhal_pwm_set_duty_cycle(&pwm_green, 100,
            PWM_LED_FREQ_HZ))
    {
        printf("PWM failed to set dutycycle!");
    }

    if(CY_RSLT_SUCCESS != cyhal_pwm_start(&pwm_green))
    {
        printf("PWM failed to start!");
    }

    /* Configure the TCPWM for driving BLUE led */
    cy_result = cyhal_pwm_init_adv(&pwm_blue, CYBSP_LED_RGB_BLUE , NC, CYHAL_PWM_RIGHT_ALIGN, true, 0u, false, NULL);
    if (CY_RSLT_SUCCESS != cy_result  )
    {
        printf("PWM initialization failed!\r\n");
    }

    if(CY_RSLT_SUCCESS != cyhal_pwm_set_duty_cycle(&pwm_blue, 100,
            PWM_LED_FREQ_HZ))
    {
        printf("PWM failed to set dutycycle!");
    }

    if(CY_RSLT_SUCCESS != cyhal_pwm_start(&pwm_blue))
    {
        printf("PWM failed to start!");
    }

    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a command has been received over queue */
        rtos_api_result = xQueueReceive(rgb_led_command_data_q, &rgb_led_cmd_data,
                portMAX_DELAY);

        /* Command has been received from queue */
        if(pdPASS == rtos_api_result)
        {
            cyhal_pwm_set_duty_cycle(&pwm_red, GET_DUTY_CYCLE((rgb_led_cmd_data.red * rgb_led_cmd_data.brightness)/100),
                    PWM_LED_FREQ_HZ);
            cyhal_pwm_set_duty_cycle(&pwm_green, GET_DUTY_CYCLE((rgb_led_cmd_data.green * rgb_led_cmd_data.brightness)/100),
                    PWM_LED_FREQ_HZ);
            cyhal_pwm_set_duty_cycle(&pwm_blue, GET_DUTY_CYCLE((rgb_led_cmd_data.blue * rgb_led_cmd_data.brightness)/100),
                    PWM_LED_FREQ_HZ);
        }
    }
}

/* END OF FILE [] */
