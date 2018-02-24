/**********************
 * Saturday Feb 24th, 2018. 9:00 PM
 *
 * This code works.
 * FIFO is ENABLED.
 * Get_status function is done, don't know how to test it.
 * Callback functions are done and tested.
 * Interrupts are integrated in this code.
 *
 *
**********************/


#include <stdint.h>
#include "M4MemMap.h"
#include "GPIO.h"
#include "GPIO_Cfg.h"
#include "UART.h"
#include "UART_Cfg.h"

// Constants to run the code
#define RX  0
#define TX  1
#define INT 0

#define RX_LEN 19
#define TX_LEN 19

uint8_t message_rx[RX_LEN+1];


int main(void)
    {



#if RX == 1

    UART_ChkType x;


    GPIO_Init();
    UART_Init();


    do
    {
    x = UART_Rx_Init(message_rx,RX_LEN,0);
    }while(x == UART_NOK);

    while(1)
    {
        UART_Rx(0);
    }

#endif

#if TX == 1

	UART_ChkType x;
	uint8_t message_tx[TX_LEN] = "How are you, mate?";

	GPIO_Init();
	UART_Init();

	do
	{
	    x = UART_Tx_Init(message_tx,TX_LEN-1,0);
	}while(x == UART_NOK);


	while(1)
	{
	    UART_Tx(0);
	}
#endif


#if INT == 1

    GPIO_Init();
    UART_Init();
    EN_INT(5);

    while(1)
    {

    }

#endif


    return 0;

    }

