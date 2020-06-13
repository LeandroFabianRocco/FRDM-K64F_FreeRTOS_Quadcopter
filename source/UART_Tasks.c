/*
 * UART_Tasks.c
 *
 *  Created on: 13 jun. 2020
 *      Author: leandro
 */



#include "UART_Tasks.h"



/*******************************************************************************
 * Variables
 ******************************************************************************/
const char *to_send               = "FreeRTOS UART driver example!\r\n";
const char *send_ring_overrun     = "\r\nRing buffer overrun!\r\n";
const char *send_hardware_overrun = "\r\nHardware buffer overrun!\r\n";
uint8_t background_buffer[32];
uint8_t recv_buffer[4];

uart_rtos_handle_t handle;
struct _uart_handle t_handle;

uart_rtos_config_t uart_config = {
    .baudrate    = 9600,
    .parity      = kUART_ParityDisabled,
    .stopbits    = kUART_OneStopBit,
    .buffer      = background_buffer,
    .buffer_size = sizeof(background_buffer),
};




/*****************************************************************************
 * @brief Task responsible for receive data from UART module
 ****************************************************************************/
void UART_Rx_Task(void *pvParameters)
{
    int error;
    int i;
    size_t n = 0;

    uart_config.srcclk = UART_CLK_FREQ;
    uart_config.base   = UART;

    if (0 > UART_RTOS_Init(&handle, &t_handle, &uart_config))
    {
        vTaskSuspend(NULL);
        PRINTF("Task suspended!\r\n");
    }

    /* Send introduction message. */
    /*if (0 > UART_RTOS_Send(&handle, (uint8_t *)to_send, strlen(to_send)))
    {
        vTaskSuspend(NULL);
    }*/

    /* Receive user input and send it back to terminal. */
    do
    {
        error = UART_RTOS_Receive(&handle, recv_buffer, sizeof(recv_buffer), &n);
        if (error == kStatus_UART_RxHardwareOverrun)
        {
            /* Notify about hardware buffer overrun */
            /*if (kStatus_Success !=
                UART_RTOS_Send(&handle, (uint8_t *)send_hardware_overrun, strlen(send_hardware_overrun)))
            {
                vTaskSuspend(NULL);
            }*/
        	PRINTF("\r\nHardware buffer overrun!\r\n");
        }
        if (error == kStatus_UART_RxRingBufferOverrun)
        {
            /* Notify about ring buffer overrun */
            /*if (kStatus_Success != UART_RTOS_Send(&handle, (uint8_t *)send_ring_overrun, strlen(send_ring_overrun)))
            {
                vTaskSuspend(NULL);
            }*/
        	PRINTF("\r\nRing buffer overrun!\r\n");
        }
        if (n > 0)
        {
            /* send back the received data */
            //UART_RTOS_Send(&handle, (uint8_t *)recv_buffer, n);
            for (i = 0; i < n; i++)
            {
            	PRINTF("0x%x\r\n", recv_buffer[i]);
            }
        }
        PRINTF("Data received!\r\n");
    } while (kStatus_Success == error);
    PRINTF("UART deinit!\r\n");
    UART_RTOS_Deinit(&handle);
    vTaskSuspend(NULL);
}
