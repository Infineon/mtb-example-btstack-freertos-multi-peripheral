/*******************************************************************************
* File Name: temperature_task.h
*
* Description: This file is the public interface of temperature_task.c source
*              file.
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
 *  Include guard
 ******************************************************************************/
#ifndef TEMPERATURE_TASK_H
#define TEMPERATURE_TASK_H

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/*******************************************************************************
 * Macro Definitions
 *******************************************************************************/
#define ABSOLUTE_ZERO      (float)(-273.15)

/** Resistance of the reference resistor */
#define R_REFERENCE        (float)(10000)

/** Beta constant of the (NCP18XH103F03RB) thermistor (3380 Kelvin).See the
 * thermistor datasheet for more details. */
#define B_CONSTANT         (float)(3380)

/** Resistance of the thermistor is 10K at 25 degrees C (from datasheet)
 * Therefore R0 = 10000 Ohm, and T0 = 298.15 Kelvin, which gives
 * R_INFINITY = R0 e^(-B_CONSTANT / T0) = 0.1192855 */
#define R_INFINITY         (float)(0.1192855)

#ifdef CYBLE_416045_02

/* Pin for the Thermistor VDD signal */
#define THERM_VDD          (P10_4)
/* Pin for the Thermistor Output option1 signal */
#define THERM_OUT1         (P10_5)
/* Pin for the Thermistor Output option2 signal */
#define THERM_OUT2         (P10_2)
/* Pin for the Thermistor Ground signal */
#define THERM_GND          (P10_3)

#elif CY8CKIT_062_BLE

/* Pin for the Thermistor VDD signal */
#define THERM_VDD          (P10_0)
/* Pin for the Thermistor Output option1 signal */
#define THERM_OUT1         (P10_1)
/* Pin for the Thermistor Output option2 signal */
#define THERM_OUT2         (P10_2)
/* Pin for the Thermistor Ground signal */
#define THERM_GND          (P10_3)

#else

/* Pin for the Thermistor VDD signal */
#define THERM_OUT1          (P3_4)
#define ADC_MAX            (56200)

#endif


/*******************************************************************************
 * Enumeration
 ******************************************************************************/
typedef enum
{
    TEMPERATURE_STOP,
    TEMPERATURE_START,
    TEMPERATURE_PROCESS
} temperature_command_t;

/*******************************************************************************
 * Global variable
 ******************************************************************************/
extern QueueHandle_t temperature_command_q;

/*******************************************************************************
 * Function prototype
 ******************************************************************************/
void task_temperature(void* param);

#endif /* CAPSENSE_TASK_H */

/* END OF FILE [] */
