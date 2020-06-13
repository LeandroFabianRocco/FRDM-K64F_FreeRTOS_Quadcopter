/*
 * UART_Tasks.h
 *
 *  Created on: 13 jun. 2020
 *      Author: leandro
 */

#ifndef UART_TASKS_H_
#define UART_TASKS_H_

#include <stdio.h>
#include "fsl_uart_freertos.h"
#include "fsl_debug_console.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define UART UART4
#define UART_CLKSRC UART4_CLK_SRC
#define UART_CLK_FREQ CLOCK_GetFreq(UART4_CLK_SRC)
#define UART_RX_TX_IRQn UART4_RX_TX_IRQn



/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void UART_Rx_Task(void *pvParameters);


#endif /* UART_TASKS_H_ */
