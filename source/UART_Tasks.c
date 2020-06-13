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
uint8_t background_buffer[22];
uint8_t recv_buffer[11];

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
    uint8_t joystick, throttle;

    uart_config.srcclk = UART_CLK_FREQ;
    uart_config.base   = UART;

    if (0 > UART_RTOS_Init(&handle, &t_handle, &uart_config))
    {
        vTaskSuspend(NULL);
        PRINTF("Task not initialized!\r\n");
    }
    for (;;){

    	PRINTF("UART_TASK\r\n");

		do
		{
			error = UART_RTOS_Receive(&handle, recv_buffer, sizeof(recv_buffer), &n);
			if (error == kStatus_UART_RxHardwareOverrun)
			{
				PRINTF("\r\nHardware buffer overrun!\r\n");
			}
			if (error == kStatus_UART_RxRingBufferOverrun)
			{
				PRINTF("\r\nRing buffer overrun!\r\n");
			}
			if (n > 0)
			{
				/*for (i = 0; i < n; i++)
				{
					PRINTF("0x%x\r\n", recv_buffer[i]);
				}*/
				//PRINTF("data_received\r\n");
				joystick = recv_buffer[7];
				throttle = recv_buffer[3];
				PRINTF("j = 0x%x; t = %3d\r\n", joystick, throttle);
			}
			PRINTF("inside loop\r\n");
		} while (kStatus_Success == error);
		//PRINTF("UART deinit!\r\n");
		//UART_RTOS_Deinit(&handle);
		//vTaskSuspend(NULL);
    }
}
