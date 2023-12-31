/*******************************************************************************
* File Name: app_bt_task.h
*
* Description: This file is the public interface of app_bt_task.c source file
*
* Related Document: README.md
*
********************************************************************************
* Copyright (2020), Cypress Semiconductor Corporation.
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
 * Include guard
 ******************************************************************************/
#ifndef APP_BT_TASK_H
#define APP_BT_TASK_H

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "wiced_bt_dev.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*******************************************************************************
* Macro Definitions
*******************************************************************************/
#define NOTIFY_TIME_MS                  (50u)

/* Connection parameters related macro */
#define CONN_INTERVAL                   (80u)
#define CONN_LATENCY                    (0u)
#define SUP_TIMEOUT                     (512u)

/*******************************************************************************
 * Data structure and enumeration
 ******************************************************************************/
typedef enum
{
    SEND_TEMP_INDICATION,           /* Send temperature indication */
    SEND_NOTIFICATION,              /* Send custom notification */
    STOP_BLE                        /* Stop Bluetooth LE */
}   app_bt_commands_list_t;

/* Data-type of Bluetooth LE commands and data */
typedef struct
{
    app_bt_commands_list_t command;
    float data;
}   app_bt_command_data_t;

/*******************************************************************************
 * Global variable
 ******************************************************************************/
extern QueueHandle_t app_bt_command_data_q;

/*******************************************************************************
 * Function prototype
 ******************************************************************************/
void task_bluetooth(void* param);

wiced_result_t app_bt_management_cb( wiced_bt_management_evt_t event,
                                      wiced_bt_management_evt_data_t *p_event_data );

#endif /* app_bt_TASK_H */

/* [] END OF FILE  */
