/*******************************************************************************
 * File Name:   app_bt_task.c
 *
 * Description: This file contains the task that handles custom Bluetooth services
 *
 * Related Document: See README.md
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
/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include <stdlib.h>
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cy_retarget_io.h"
#include "cycfg_gap.h"
#include "cycfg_gatt_db.h"
#include "cycfg_bt_settings.h"
#include "wiced_bt_stack.h"
#include "rgb_led_task.h"
#include "temperature_task.h"
#include "app_bt_task.h"
#include "wiced_bt_gatt.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <string.h>
#include <timers.h>
#include <math.h>
#include <wiced_bt_l2c.h>

/*******************************************************************************
 * Macro Definitions
 *******************************************************************************/
#define ENABLED                     (1u)
#define DISABLED                    (0u)
/* Used to enable or disable UART DEBUG Logs */
#define DEBUG_UART_ENABLED          (DISABLED)

/* Total size of the Health Thermometer Service characteristic array */
#define HTS_CHARACTERISTIC_SIZE     (uint8_t) (5u)
/* Size of the temperature value in the HTS characteristic array */
#define HTS_TEMPERATURE_DATA_SIZE   (uint8_t) (4u)
/* Index of the temperature value in the HTS characteristic array */
#define HTS_TEMPERATURE_DATA_INDEX  (uint8_t) (1u)

/**
 * Macros used to convert from IEEE-754 single precision floating
 * point format to IEEE-11073 FLOAT with two decimal digits of precision
 */
#define IEEE_11073_MANTISSA_SCALER  (uint8_t) (100u)
#define IEEE_11073_EXPONENT_VALUE   (int8_t)  (-2)
#define IEEE_11073_EXPONENT_INDEX   (uint8_t) (3u)


#if (DEBUG_UART_ENABLED == ENABLED)
#define DBG_PRINTF(...)                 (printf(__VA_ARGS__))
#else
#define DBG_PRINTF(...)
#endif

/*******************************************************************************
 * Structure
 ******************************************************************************/

/**
 * Data-type used to store temperature as an IEEE-11073 FLOAT value as well as
 *  access it as an array of bytes for Bluetooth LE operations
 */
typedef union
{
    int32_t temeratureValue;
    int8_t  temperatureArray[HTS_TEMPERATURE_DATA_SIZE];
} temperature_data_t;

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
static void app_bt_init(void);
static wiced_bt_gatt_status_t app_bt_gatt_event_handler( wiced_bt_gatt_evt_t event,
        wiced_bt_gatt_event_data_t *p_data);
static wiced_bt_gatt_status_t app_bt_gatt_read_handler( uint16_t conn_id,
        wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_read_t *p_read_req,
        uint16_t req_len);
static wiced_bt_gatt_status_t app_bt_gatt_write_handler( uint16_t conn_id,
        wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_write_req_t *p_write_req,
        uint16_t req_len);
static wiced_bt_gatt_status_t app_bt_gatt_req_write_value(uint16_t attr_handle, uint8_t *p_val,
        uint16_t len);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler(uint16_t conn_id,
        wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_read_by_type_t *p_read_req,
        uint16_t req_len);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_multi_handler(uint16_t conn_id,
        wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_read_multiple_req_t *p_read_req,
        uint16_t req_len);
static gatt_db_lookup_table_t  *app_bt_find_by_handle(uint16_t handle);
static void app_bt_print_bd_address(wiced_bt_device_address_t bdadr);
static void send_temperature_indication(float temperature);
static void send_custom_notification(void);

/*******************************************************************************
 * Global variable
 ******************************************************************************/
/* Queue handle for app bt data */
QueueHandle_t app_bt_command_data_q;

/* Holds the connection ID */
uint16_t app_bt_dev_connection_id[4] = {0};

uint8_t custom_notification_enabled = 0;
uint16_t notification_id = 0;
uint8_t custom_notification_data[3];

/* Variable used to manage HTS request count */
uint8_t hts_request_count = 0;
/* Variable used to manage HTS */
uint8_t hts_indication = 0;

/* Holds the app bt data  */
app_bt_command_data_t app_bt_command;

/**
 * @brief Typdef for function used to free allocated buffer to stack
 */
typedef void (*pfn_free_buffer_t)(uint8_t *);

typedef struct
{
    uint8_t     valueArray[4];
    uint32_t    colorAndIntensity;
}rgb_led_data_t;

/* RGB LED array */
uint8_t rgb_led[4]={0};

/*******************************************************************************
 * Function Name: task_bluetooth
 ********************************************************************************
 * Summary:
 *  Task that processes the Bluetooth LE state and events, and then commands other 
 *  tasks to take an action based on the current Bluetooth LE state and data
 *  received over Bluetooth LE
 * 
 * Parameters:
 *  void *param : Task parameter defined during task creation (unused)
 *
 *******************************************************************************/
void task_bluetooth(void* param)
{
    BaseType_t rtos_api_result = pdFAIL;

    /* Suppress warning for unused parameter */
    (void)param;

    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a command has been received over queue */
        rtos_api_result = xQueueReceive(app_bt_command_data_q, &app_bt_command,
                portMAX_DELAY);

        /* Command has been received from queue */
        if(pdPASS == rtos_api_result)
        {

            switch(app_bt_command.command)
            {
            /* Command to send temperature indication */
            case SEND_TEMP_INDICATION:
            {
                /* Send temperature data over Health Thermometer notification */
                send_temperature_indication(app_bt_command.data);
                break;
            }

            /* Command to send notification */
            case SEND_NOTIFICATION:
            {
                /* Send notification to all the connected devices */
                send_custom_notification();
                break;
            }

            /* Command to stop BLE */
            case STOP_BLE:
            {
                /* Stop the BLE and update the LED status*/
                break;
            }

            /* Unknown command */
            default:
            {
                break;
            }
            }
        }
    }
}

/*******************************************************************************
 * Function Name: app_bt_management_cb
 ********************************************************************************
 * Summary:
 * This function handles the BT stack events.
 *
 * Parameters:
 *  wiced_bt_management_evt_t event: event code
 *  wiced_bt_management_evt_data_t *p_event_data : Data corresponding to the event
 *
 * Return:
 *  wiced_result_t : status
 *
 *******************************************************************************/

wiced_result_t app_bt_management_cb( wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = { 0 };
    DBG_PRINTF("App management cback: 0x%x\r\n", event);
    switch (event)
    {
    case BTM_ENABLED_EVT:
        /* Bluetooth Controller and Host Stack Enabled */
        if (WICED_BT_SUCCESS == p_event_data->enabled.status)
        {
            wiced_bt_dev_read_local_addr(bda);
            printf("Bluetooth local device address: ");
            app_bt_print_bd_address(bda);

            /* Perform application-specific initialization */
            app_bt_init();
        }
        break;
    case BTM_PIN_REQUEST_EVT:
    case BTM_PASSKEY_REQUEST_EVT:
        status = WICED_BT_ERROR;
        break;
    case BTM_DISABLED_EVT:
        break;
    case BTM_BLE_CONNECTION_PARAM_UPDATE:

        status = WICED_SUCCESS;
        break;
    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:

        /* Advertisement State Changed */
        printf("Advertisement state change: 0x%x\r\n",
                p_event_data->ble_advert_state_changed);
        break;

    default:
        break;
    }

    return status;
}


/*******************************************************************************
 * Function Name: app_bt_gatt_conn_status_cb
 ********************************************************************************
 * Summary:
 * This function is invoked on GATT connection event
 *
 * Parameters:
 *  wiced_bt_gatt_connection_status_t *p_conn_status : GATT connection status
 *
 * Return:
 *  wiced_bt_gatt_status_t status : Status
 *
 *******************************************************************************/

wiced_bt_gatt_status_t app_bt_gatt_conn_status_cb(
        wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_result_t result = WICED_BT_ERROR;

        if (p_conn_status->connected)
        {
            /* Device has connected */
            printf("Bluetooth connected with device address:" );
            app_bt_print_bd_address(p_conn_status->bd_addr);

            for(uint8 i=0;i<4;i++)
            {
                if(app_bt_dev_connection_id[i] == 0)
                {
                    app_bt_dev_connection_id[i] = p_conn_status->conn_id;
                    break;
                }
            }
            /* Stop Advertisement */
            wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );
            /* Start Advertisement */
            wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
        }
        else
        {
            /* Device has disconnected */
            printf("Bluetooth disconnected with device address:" );
            app_bt_print_bd_address(p_conn_status->bd_addr);
            DBG_PRINTF("Bluetooth device connection id: 0x%x\r\n", p_conn_status->conn_id );
            for(uint8 i=0;i<4;i++)
            {
                if(app_bt_dev_connection_id[i] == p_conn_status->conn_id)
                {
                    /* Set the connection id to zero to indicate disconnected state */
                    app_bt_dev_connection_id[i] = 0;
                }
            }

            /* Restart the advertisements */
            result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
            /* Failed to start advertisement. Stop program execution */
            if (CY_RSLT_SUCCESS != result)
            {
                CY_ASSERT(0);
            }
        }
        status = WICED_BT_GATT_SUCCESS;
    return status;
}


/*******************************************************************************
 * Function Name: app_bt_init
 ********************************************************************************
 * Summary:
 * This function handles application level initialization tasks and is called from
 * the BT management callback once the BLE stack enabled event BTM_ENABLED_EVT is
 * triggered. This function is executed in the BTM_ENABLED_EVT management callback.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 *******************************************************************************/
static void app_bt_init( void )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_result_t result = WICED_BT_ERROR;

    DBG_PRINTF("Discover the device with name: \"%s\"\r\n\r\n", app_gap_device_name);
    /* Register with BT stack to receive GATT callback */
    status = wiced_bt_gatt_register(app_bt_gatt_event_handler);
    DBG_PRINTF("GATT event handler registration status: 0x%x\r\n", status);

    /* Initialize GATT Database */
    status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
    DBG_PRINTF("GATT database initiliazation status: 0x%x\r\n", status);

    /* Disable pairing for this application */
    wiced_bt_set_pairable_mode(WICED_FALSE, WICED_FALSE);

    /* Set Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE, cy_bt_adv_packet_data);

    /* Start Undirected LE Advertisements on device startup.
     * The corresponding parameters are contained in 'app_bt_cfg.c' */
    result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

    /* Failed to start advertisement. Stop program execution */
    if (WICED_BT_SUCCESS != result)
    {
        DBG_PRINTF("Failed to start advertisement! \n");
        CY_ASSERT(0);
    }
    UNUSED_VARIABLE(status);
}

/*******************************************************************************
 * Function Name : app_bt_alloc_buffer
 *******************************************************************************
 *
 * Summary:
 * @brief  This Function allocates the buffer of requested length
 *
 * @param len            Length of the buffer
 *
 * @return uint8_t*      pointer to allocated buffer
 ******************************************************************************/
static uint8_t *app_bt_alloc_buffer(uint16_t len)
{
    uint8_t *p = (uint8_t *)malloc(len);
    return p;
}

/*******************************************************************************
 * Function Name : app_bt_free_buffer
 *******************************************************************************
 * Summary :
 * @brief  This Function frees the buffer requested
 *
 * @param p_data         pointer to the buffer to be freed
 *
 * @return void
 ******************************************************************************/
static void app_bt_free_buffer(uint8_t *p_data)
{
    if (NULL != p_data )
    {
        free(p_data);
    }
}

/*******************************************************************************
 * Function Name: app_bt_gatt_event_handler
 ********************************************************************************
 * Summary:
 * This function handles GATT callback events from the BT stack.
 *
 * Parameters:
 *  wiced_bt_gatt_evt_t event                   : LE GATT event code
 *  wiced_bt_gatt_event_data_t *p_event_data    : Pointer to LE GATT event data
 *
 * Return:
 *  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
 *  in wiced_bt_gatt.h
 *
 ********************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_event_handler( wiced_bt_gatt_evt_t event,
        wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_bt_gatt_attribute_request_t *p_attr_req = &p_data->attribute_request;

    /* Call the appropriate callback function based on the GATT event type, and pass the relevant event
     * parameters to the callback function */
    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        status = app_bt_gatt_conn_status_cb(&p_data->connection_status);
        if(WICED_BT_GATT_SUCCESS != status)
        {
            DBG_PRINTF("GATT connection cb failed: 0x%x\r\n", status);
        }
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        switch(p_attr_req->opcode)
        {
        case GATT_REQ_READ:
            status = app_bt_gatt_read_handler(p_attr_req->conn_id,p_attr_req->opcode,
                    &p_attr_req->data.read_req,p_attr_req->len_requested);
            break;

        case GATT_REQ_READ_BY_TYPE:
            status = app_bt_gatt_req_read_by_type_handler(p_attr_req->conn_id, p_attr_req->opcode,
                    &p_attr_req->data.read_by_type,
                    p_attr_req->len_requested);
            break;
        case GATT_REQ_READ_MULTI:
        case GATT_REQ_READ_MULTI_VAR_LENGTH:
            status = app_bt_gatt_req_read_multi_handler(p_attr_req->conn_id, p_attr_req->opcode,
                    &p_attr_req->data.read_multiple_req,
                    p_attr_req->len_requested);
            break;

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
        case GATT_CMD_SIGNED_WRITE:
            status = app_bt_gatt_write_handler(p_attr_req->conn_id,p_attr_req->opcode,
                    &p_attr_req->data.write_req,
                    p_attr_req->len_requested);

            if ((GATT_REQ_WRITE == p_attr_req->opcode) && (WICED_BT_GATT_SUCCESS == status ))
            {
                DBG_PRINTF("\r\n write request received \r\n");
                wiced_bt_gatt_write_req_t *p_write_request = &p_attr_req->data.write_req;
                wiced_bt_gatt_server_send_write_rsp(p_attr_req->conn_id, p_attr_req->opcode,
                        p_write_request->handle);
            }

            break;
        case GATT_REQ_MTU:
            DBG_PRINTF("GATT req mtu \r\n");
            status = wiced_bt_gatt_server_send_mtu_rsp(p_attr_req->conn_id,
                    p_attr_req->data.remote_mtu,
                    wiced_bt_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size);
            break;
        }
        break;
        case GATT_GET_RESPONSE_BUFFER_EVT: /* GATT buffer request, typically sized to max of bearer mtu - 1 */
            p_data->buffer_request.buffer.p_app_rsp_buffer = app_bt_alloc_buffer
            (p_data->buffer_request.len_requested);
            p_data->buffer_request.buffer.p_app_ctxt = (void *)&app_bt_free_buffer;
            status = WICED_BT_GATT_SUCCESS;
            break;
        case GATT_APP_BUFFER_TRANSMITTED_EVT: /* GATT buffer transmitted event,  check \ref wiced_bt_gatt_buffer_transmitted_t*/
        {
            pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to free it. */
            if (pfn_free)
            {
                pfn_free(p_data->buffer_xmitted.p_app_data);
                status = WICED_BT_GATT_SUCCESS;
            }
        }
        break;
        default:
                printf("ERROR: Unhandled GATT Connection Request case: %d\n",
                       p_attr_req->opcode);
                break;

    }
    return status;
}

/*******************************************************************************
 * Function Name: app_bt_gatt_req_write_value
 ********************************************************************************
 * Summary:
 * This function handles writing to the attribute handle in the GATT database
 * using the data passed from the BT stack. The value to write is stored in a
 * buffer whose starting address is passed as one of the function parameters
 *
 * Parameters:
 * @param attr_handle  GATT attribute handle
 * @param p_val        Pointer to LE GATT write request value
 * @param len          length of GATT write request
 * Return:
 *  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
 *  in wiced_bt_gatt.h
 *
 *******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_req_write_value(uint16_t attr_handle, uint8_t *p_val, uint16_t len)
{
    wiced_bt_gatt_status_t result  = WICED_BT_GATT_INVALID_HANDLE;


    rgb_led_command_data_t led_cmd_data;

    /* Check for a matching handle entry */
    for (int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {

            if (app_gatt_db_ext_attr_tbl[i].max_len >= len)
            {
                /* Value fits within the supplied buffer; copy over the value */
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memset(app_gatt_db_ext_attr_tbl[i].p_data, 0x00, app_gatt_db_ext_attr_tbl[i].max_len);
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, app_gatt_db_ext_attr_tbl[i].cur_len);

                if (0 == memcmp(app_gatt_db_ext_attr_tbl[i].p_data, p_val, app_gatt_db_ext_attr_tbl[i].cur_len))
                {
                    result = WICED_BT_GATT_SUCCESS;
                }

                /* Add code for any action required when this attribute is written.
                 * In this case, we initilize the characteristic value */

                switch ( attr_handle )
                {
                case HDLD_CUSTOM_NOTIFICATION_CUSTOM_CHARACTERISTIC_CLIENT_CHAR_CONFIG:

                    custom_notification_enabled = p_val[0];

                    break;

                case HDLC_CUSTOM_SERVICE_CUSTOM_CHARACTERISTIC_VALUE:

                    custom_notification_data[0] = notification_id >> 8;
                    custom_notification_data[1] = notification_id;
                    custom_notification_data[2] = 02;

                    break;

                case HDLC_RGB_LED_RGB_LED_CONTROL_VALUE:

                    led_cmd_data.brightness =  (p_val[3]*100)/255;
                    led_cmd_data.red=  (p_val[0]*100)/255;
                    led_cmd_data.green =  (p_val[1]*100)/255;
                    led_cmd_data.blue =  (p_val[2]*100)/255;

                    xQueueSendToBack(rgb_led_command_data_q, &led_cmd_data, 0u);

                    custom_notification_data[0] = notification_id >> 8;
                    custom_notification_data[1] = notification_id;
                    custom_notification_data[2] = 01;

                    break;

                case HDLD_HTS_TEMPERATURE_MEASUREMENT_CLIENT_CHAR_CONFIG:

                    if(p_val[0])
                    {
                        /* Tracking how many devices requested for HTS */
                        hts_request_count ++;
                        /* Set the hts_indication flag */
                        hts_indication = true;
                    }
                    else
                    {
                        hts_request_count --;
                        if(hts_request_count == 0)
                        {
                            /* Reset the hts_indication flag */
                            hts_indication = false;
                        }
                    }

                    if(hts_indication)
                    {
                        /* Start the timer */
                        temperature_command_t command = TEMPERATURE_START;
                        xQueueSend(temperature_command_q, &command, 0u);
                    }
                    else
                    {
                        /* Stop the timer */
                        temperature_command_t command = TEMPERATURE_STOP;
                        xQueueSend(temperature_command_q, &command, 0u);
                    }
                    break;
                                    
                 default:
                    result = WICED_BT_GATT_INVALID_HANDLE;
                    break;
                }

                /* Queue app bt command to send notification to all the connected devices */
                app_bt_command_data_t app_bt_command = {.command = SEND_NOTIFICATION};
                xQueueSendToBack(app_bt_command_data_q, &app_bt_command, 0u);
            }
            else
            {
                /* Value to write does not meet size constraints */
                result = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
            
        }
    }

    if (WICED_BT_GATT_SUCCESS != result)
    {
        DBG_PRINTF("Write request to invalid handle: 0x%x\r\n", attr_handle);
    }

    return result;
}

/*******************************************************************************
 * Function Name: app_bt_gatt_write_handler
 ********************************************************************************
 * Summary:
 * This function handles Write Requests received from the client device
 *
 * Parameters:
 * @param conn_id         Connection ID
 * @param opcode          LE GATT request type opcode
 * @param p_write_req     Pointer that contains details of Write
 *                        Request including the attribute handle
 * @param req_len   length of data requested
 * Return:
 *  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
 *  in wiced_bt_gatt.h
 *
 *******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_write_handler( uint16_t conn_id,
        wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_write_req_t *p_write_req,
        uint16_t req_len)
{
    gatt_db_lookup_table_t  *puAttribute = NULL;

    puAttribute = app_bt_find_by_handle(p_write_req->handle);
    if ( NULL == puAttribute )
    {
        DBG_PRINTF(" Attribute not found, Handle: 0x%04x\r\n", p_write_req->handle);
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_write_req->handle, WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    notification_id = conn_id;
    app_bt_gatt_req_write_value(p_write_req->handle, p_write_req->p_val, p_write_req->val_len);

    return WICED_BT_GATT_SUCCESS;
}


/*******************************************************************************
 * Function Name: app_bt_gatt_read_handler
 ********************************************************************************
 * Summary:This function handles Read Requests received from the client device.
 *
 * Parameters:
 * @param conn_id       Connection ID
 * @param opcode        LE GATT request type opcode
 * @param p_read_req    Pointer to read request containing the handle to read
 * @param req_len length of data requested
 *
 * Return:
 *  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
 *  in wiced_bt_gatt.h
 *
 *******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_read_handler( uint16_t conn_id,
        wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_read_t *p_read_req,
        uint16_t req_len)
{
    gatt_db_lookup_table_t  *puAttribute = NULL;
    int          attr_len_to_copy;
    uint8_t     *from;
    uint16_t     to_send;

    puAttribute = app_bt_find_by_handle(p_read_req->handle);
    if ( NULL == puAttribute )
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    attr_len_to_copy = puAttribute->cur_len;
    if (p_read_req->offset >= puAttribute->cur_len)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                WICED_BT_GATT_INVALID_OFFSET);
        return WICED_BT_GATT_INVALID_OFFSET;
    }
    to_send = MIN(req_len, attr_len_to_copy - p_read_req->offset);
    from = ((uint8_t *)puAttribute->p_data) + p_read_req->offset;
    wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL);
    return wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL); /* No need for context, as buff not allocated */;

}

/*******************************************************************************
 * Function Name: app_bt_gatt_req_read_by_type_handler
 ********************************************************************************
 * Summary:@brief  Process read-by-type request from peer device
 *
 * @param conn_id       Connection ID
 * @param opcode        LE GATT request type opcode
 * @param p_read_req    Pointer to read request containing the handle to read
 * @param req_len length of data requested
 *
 * @return wiced_bt_gatt_status_t  LE GATT status
 *******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler(uint16_t conn_id,
        wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_read_by_type_t *p_read_req,
        uint16_t req_len)
{
    gatt_db_lookup_table_t *puAttribute = NULL;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = app_bt_alloc_buffer(req_len);
    uint8_t pair_len = 0;
    uint16_t used_len = 0;
    if ( NULL == p_rsp )
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, attr_handle, WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }
    /* Read by type returns all attributes of the specified type, between the start and end handles */
    while (WICED_TRUE)
    {
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle, p_read_req->e_handle,
                &p_read_req->uuid);

        if (0 == attr_handle)
            break;

        puAttribute = app_bt_find_by_handle(attr_handle);
        if ( NULL == puAttribute )
        {
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle,
                    WICED_BT_GATT_ERR_UNLIKELY);
            app_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        {
            int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used_len,
                    req_len - used_len, &pair_len,
                    attr_handle, puAttribute->cur_len,
                    puAttribute->p_data);
            if (0 == filled)
            {
                break;
            }
            used_len += filled;
        }
        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }
    if ( !used_len )
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle,
                WICED_BT_GATT_INVALID_HANDLE);
        app_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp(conn_id, opcode, pair_len, used_len, p_rsp,
            (void *)&app_bt_free_buffer);
    return WICED_BT_GATT_SUCCESS;
}


/*******************************************************************************
 * Function Name: app_bt_gatt_req_read_multi_handler
 ********************************************************************************
 * Summary : @brief  Process write read multi request from peer device
 *
 * @param conn_id       Connection ID
 * @param opcode        LE GATT request type opcode
 * @param p_read_req    Pointer to read request containing the handle to read
 * @param req_len length of data requested
 *
 * @return wiced_bt_gatt_status_t  LE GATT status
 ******************************************************************************/
static wiced_bt_gatt_status_t app_bt_gatt_req_read_multi_handler(uint16_t conn_id,
        wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_read_multiple_req_t *p_read_req,
        uint16_t req_len)
{
    gatt_db_lookup_table_t *puAttribute = NULL;
    uint8_t *p_rsp = app_bt_alloc_buffer(req_len);
    uint16_t used_len = 0;
    uint16_t count;
    uint16_t handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, 0);
    if ( NULL ==p_rsp )
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, handle, WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    /* Read by type returns all attributes of the specified type, between the start and end handles */
    for (count = 0; count < p_read_req->num_handles; count++)
    {
        handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, count);
        puAttribute = app_bt_find_by_handle(handle);
        if ( NULL == puAttribute )
        {
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, *p_read_req->p_handle_stream,
                    WICED_BT_GATT_ERR_UNLIKELY);
            app_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_ERR_UNLIKELY;
        }
        else
        {
            int filled = wiced_bt_gatt_put_read_multi_rsp_in_stream(opcode, p_rsp + used_len,
                    req_len - used_len,
                    puAttribute->handle,
                    puAttribute->cur_len,
                    puAttribute->p_data);
            if (!filled)
            {
                break;
            }
            used_len += filled;
        }
    }
    if ( !used_len )
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, *p_read_req->p_handle_stream,
                WICED_BT_GATT_INVALID_HANDLE);
        app_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    /* Send the response */
    wiced_bt_gatt_server_send_read_multiple_rsp(conn_id, opcode, used_len, p_rsp,
            (void *)&app_bt_free_buffer);
    return WICED_BT_GATT_SUCCESS;
}


/**************************************************************************************************
 * Function Name: send_custom_notification
 ***************************************************************************************************
 * Summary:
 *      Send the notification message to the client.
 *
 * Parameters:
 *   None
 *
 * Return:
 *  None
 *
 *************************************************************************************************/
static void send_custom_notification(void)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    if( (custom_notification_enabled))
    {
        for(uint8_t i=0;i<4;i++)
        {
            status = wiced_bt_gatt_server_send_notification(
                    app_bt_dev_connection_id[i],
                    HDLC_CUSTOM_NOTIFICATION_CUSTOM_CHARACTERISTIC_VALUE,
                    3,
                    custom_notification_data,NULL);

            if(WICED_BT_GATT_SUCCESS == status)
            {
                DBG_PRINTF("Sending custom notificaiton FAILED\r\n");
            }
        }
    }
}


/**************************************************************************************************
 * Function Name: send_temperature_indication
 ***************************************************************************************************
 * Summary:
 *      Send the temperature data to the client through indications.
 *
 * Parameters:
 *   None
 *
 * Return:
 *  None
 *
 *************************************************************************************************/
static void send_temperature_indication(float temperature)
{
    /* Temporary array to hold Health Thermometer Characteristic information */
    uint8 valueArray[HTS_CHARACTERISTIC_SIZE];
    temperature_data_t tempData;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    /* Convert from IEEE-754 single precision floating point format to
       IEEE-11073 FLOAT, which is mandated by the health thermometer
       characteristic */
    tempData.temeratureValue = (int32_t)(roundf(temperature*
            IEEE_11073_MANTISSA_SCALER));
    tempData.temperatureArray[IEEE_11073_EXPONENT_INDEX] =
            IEEE_11073_EXPONENT_VALUE;

    /* Update temperature value in the characteristic */
    memcpy(&valueArray[HTS_TEMPERATURE_DATA_INDEX],
            tempData.temperatureArray, HTS_TEMPERATURE_DATA_SIZE);

    for(uint8 i=0;i<4;i++)
    {
        status = wiced_bt_gatt_server_send_indication(
                app_bt_dev_connection_id[i],
                HDLC_HTS_TEMPERATURE_MEASUREMENT_VALUE,
                HTS_CHARACTERISTIC_SIZE,
                valueArray,NULL);

        if(WICED_BT_GATT_SUCCESS != status)
        {
            DBG_PRINTF("Sending Temperature Indication FAILED\r\n");
        }
    }

}

/*******************************************************************************
 * Function Name: app_bt_print_bd_address
 ********************************************************************************
 * Summary: This is the utility function that prints the address of the Bluetooth device
 *
 * Parameters:
 *  wiced_bt_device_address_t bdaddr : Bluetooth address
 *
 * Return:
 *  None
 *
 *******************************************************************************/
static void app_bt_print_bd_address(wiced_bt_device_address_t bdadr)
{
    for(uint8_t i=0;i<BD_ADDR_LEN-1;i++)
    {
        printf("%2X:",bdadr[i]);
    }
    printf("%2X\n",bdadr[BD_ADDR_LEN-1]);
}

/*******************************************************************************
 * Function Name : app_bt_find_by_handle
 * *****************************************************************************
 * Summary : @brief  Find attribute description by handle
 *
 * @param handle    handle to look up
 *
 * @return gatt_db_lookup_table_t   pointer containing handle data
 ******************************************************************************/
static gatt_db_lookup_table_t  *app_bt_find_by_handle(uint16_t handle)
{
    int i;
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (handle == app_gatt_db_ext_attr_tbl[i].handle )
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}
/* END OF FILE [] */
