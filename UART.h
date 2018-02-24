#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "UART_Cfg.h"


typedef enum {UART_OK = 0, UART_NOK = 1} UART_ChkType;
typedef enum {OneStopBit = 0, TwoStopBit = 1} UART_StopBitsType;
typedef enum {Data_5 = 0, Data_6 = 1, Data_7 = 2, Data_8 = 3} UART_WordLength;
typedef enum {Disabled = 0, Enabled = 1} UART_EnableType;
typedef enum {Idle = 0, TxInProg = 1, TxDone = 2, RxInProg = 3, RxDone = 4} UART_StatusChkType;
typedef enum {FIFO_2=0, FIFO_4=0x1, FIFO_8=0x2, FIFO_12=0x3, FIFO_14=0x4}FIFO_INT_Level ;

typedef void(*UART_PtrToCallbackType)(void);


/* A structure type contains all the required configurations */

typedef struct
{

    /* UART port number */
    uint8_t UARTPortID;

    /* Tx pin number */
    uint8_t TxPinID;

    /* Rx pin number */
    uint8_t RxPinID;

    /* Baud rate */
    uint32_t BaudRate;

    /* Number of stop bits */
    UART_StopBitsType StopBits;

    /* Word length */
    UART_WordLength WordLen;

    /* Parity */
    UART_EnableType Parity;

    /* Enabling FIFO */
    UART_EnableType FIFOEN;

    /* Tx FIFO size */
    uint8_t TxFIFOSize;

    /* Rx FIFO size */
    uint8_t RxFIFOSize;

    /* TxDone Callback function pointer */
    UART_PtrToCallbackType TxDoneCallbackPtr;

    /* RxDone Callback function pointer */
    UART_PtrToCallbackType RxDoneCallbackPtr;

    /* Enable and Disable interrupt*/
    UART_EnableType INT_Enable;

    /* Interrupt FIFo level*/
    FIFO_INT_Level FIFO_Level;


} UART_ConfigType;

extern const UART_ConfigType UART_ConfigParam[UART_GROUPS_NUMBER];

/* Declaration of global functions */
UART_ChkType UART_Init (void);
UART_ChkType UART_Tx_Init(const uint8_t* TxBuffPtr, uint8_t TxLen, uint8_t groupID);
UART_ChkType UART_Tx (uint8_t groupID);
UART_ChkType UART_Rx_Init (const uint8_t* RxBuffPtr, uint8_t RxLen, uint8_t groupId);
UART_ChkType UART_Rx (uint8_t groupId);
UART_StatusChkType UART_Get_Status (uint8_t groupID);

void TxDone_Func (void);
void RxDone_Func (void);



#endif
