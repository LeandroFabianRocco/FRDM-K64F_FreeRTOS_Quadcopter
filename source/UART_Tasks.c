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
uint8_t background_buffer[30];
uint8_t recv_buffer[3];

uart_rtos_handle_t handle;
struct _uart_handle t_handle;

uart_rtos_config_t uart_config = {
    .baudrate    = 9600,
    .parity      = kUART_ParityDisabled,
    .stopbits    = kUART_OneStopBit,
    .buffer      = background_buffer,
    .buffer_size = sizeof(background_buffer),
};


// CRC-8 table
static uint8_t crc8_table[256];



/*******************************************************************************
 * Initialization of CRC-8 table
 ******************************************************************************/
static void init_crc8(void)
{
    int divident;
    uint8_t bit;

    for (divident=0; divident < 256; divident++)
    {
        uint8_t currByte = (uint8_t)divident;
        for (bit=0; bit<8; bit++)
        {
            if ((currByte & 0x80) != 0)
            {
                currByte <<= 1;
                currByte ^= GENERATOR;
            }
            else
            {
                currByte <<= 1;
            }
        }
        crc8_table[divident] = currByte;
    }
}


/*******************************************************************************
 * Computation of CRC-8 code
 ******************************************************************************/
uint8_t Compute_CRC8(uint8_t *data, uint8_t size)
{
    uint8_t crc = 0x00;
    uint8_t i;
    uint8_t aux;
    for (i = 0; i < size; i++)
    {
        aux = *data ^ crc;
        crc = crc8_table[aux];
        *data++;
    }

    return crc;
}



/*****************************************************************************
 * @brief Task responsible for receive data from UART module
 ****************************************************************************/
void UART_Rx_Task(void *pvParameters)
{
    int error;
    //int i;
    size_t n = 0;

    uint8_t rest;

    uart_config.srcclk = UART_CLK_FREQ;
    uart_config.base   = UART;

    // Initialization of CRC-8 table
    init_crc8();

    if (0 > UART_RTOS_Init(&handle, &t_handle, &uart_config))
    {
        vTaskSuspend(NULL);
    }

    // Motors data
    Attitude_Joystick_Data_t motor_data;
    motor_data.evPitch = FALSE;
	motor_data.evRoll = FALSE;
	motor_data.evYaw = FALSE;
	motor_data.evJandT = TRUE;

	motor_data.eThrottle = 0;
	motor_data.eJoystick = 0;
	motor_data.ePidPitch = 0;
	motor_data.ePidRoll = 0;
	motor_data.ePidYaw = 0;

    for (;;)
    {
    	GPIO_PortClear(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_PIN);
		GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_PIN);
		GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_PIN);
		PRINTF("---> UART_Task!!\r\n");
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
				rest = Compute_CRC8(recv_buffer, sizeof(recv_buffer));
				if (rest == 0) // CRC checked!
				{
					if (recv_buffer[0] == 0x2A)
					{
						motor_data.eThrottle = recv_buffer[1];
					}
					else if (recv_buffer[0] == 0x23)
					{
						motor_data.eJoystick = recv_buffer[1];
					}
				}
				//PRINTF("j = 0x%x; t = %3d\r\n", motor_data.eJoystick, motor_data.eThrottle);
				// Send data to queue
				//xQueueSend(motors_queue, &motor_data, 0);
				if (xQueueSend(motors_queue, &motor_data, 0) == errQUEUE_FULL)
				{
					PRINTF("Queue is FULL!!!\r\n");
				}
				else
					PRINTF("Queue writed!!!\r\n");
			}
		} while (kStatus_Success == error);
    }
}
