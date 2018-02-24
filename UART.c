#include "UART.h"
#include "UART_Cfg.h"
#include "M4MemMap.h"
#include <stdint.h>

typedef volatile uint32_t* const UART_RegAddType;

#define PORTS_NUMBER 8U

#define LOOPBACKMODE 0


/* Registers memory map */


#define UART0_BASE_ADDRESS 0x4000C000
#define UART1_BASE_ADDRESS 0x4000D000
#define UART2_BASE_ADDRESS 0x4000E000
#define UART3_BASE_ADDRESS 0x4000F000
#define UART4_BASE_ADDRESS 0x40010000
#define UART5_BASE_ADDRESS 0x40011000
#define UART6_BASE_ADDRESS 0x40012000
#define UART7_BASE_ADDRESS 0x40013000



static const uint32_t PortsBaseAddressLut[PORTS_NUMBER]
={
  UART0_BASE_ADDRESS,
  UART1_BASE_ADDRESS,
  UART2_BASE_ADDRESS,
  UART3_BASE_ADDRESS,
  UART4_BASE_ADDRESS,
  UART5_BASE_ADDRESS,
  UART6_BASE_ADDRESS,
  UART7_BASE_ADDRESS
};


#define UART_REG_ADDRESS(ID,REG_OFFSET) (PortsBaseAddressLut[ID] + REG_OFFSET)


/* Data Control */
#define UARTDR_REG(PORT_ID)             *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x000))


/* Clock Control */

#define UARTCC_REG(PORT_ID)             *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0xFC8))
#define UARTCTL_REG(PORT_ID)            *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x030))


/* DMA Control */

#define UARTDMACTL_REG(PORT_ID)         *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x048))


/* Interrupt Control */

#define UARTIFLS_REG(PORT_ID)           *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x034))
#define UARTIM_REG(PORT_ID)             *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x038))
#define UARTRIS_REG(PORT_ID)            *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x03C))
#define UARTMIS_REG(PORT_ID)            *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x040))
#define UARTICR_REG(PORT_ID)            *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x044))


/* Status Control */

#define UARTRSR_REG(PORT_ID)            *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x004))
#define UARTFR_REG(PORT_ID)             *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x018))
#define UARTLCRH_REG(PORT_ID)           *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x02C))
#define UARTILPR_REG(PORT_ID)           *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x020))
#define UART9BITADDR_REG(PORT_ID)       *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x0A4))
#define UART9BITAMASK_REG(PORT_ID)      *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x0A8))
#define UARTPP_REG(PORT_ID)             *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0xFC0))


/* Baud Rate Control */

#define UARTIBRD_REG(PORT_ID)           *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x024))
#define UARTFBRD_REG(PORT_ID)           *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x028))


/* Defining some constants */

#define SYSCLK          16000000.0

#define PEN_BIT_NO      1U
#define STP2_BIT_NO     3U
#define FEN_BIT_NO      4U
#define WLEN_BIT_NO     5U

#define UARTEN_BIT_NO   0U
#define LBE_BIT_NO      7U
#define TXE_BIT_NO      8U
#define RXE_BIT_NO      9U

#define BUSY_BIT_NO     3U
#define RXFE_BIT_NO     4U
#define TXFF_BIT_NO     5U
#define TXFE_BIT_NO     7U

//Interrupt bits definition
#define RXIM_BIT_NO     4U
#define RXIFLSEL_BIT_NO 3U
#define RXIC_BIT_NO     4U


#define INT_BUFF_LEN 19
uint8_t INT_msg_rx_buff[INT_BUFF_LEN];


// Defining some variables

static uint8_t UART_Driver_State[UART_GROUPS_NUMBER] = {0};

#define UART_INIT_DONE 0U
#define UART_TX_INIT_DONE 1U
#define UART_TX_DONE 2U
#define UART_RX_INIT_DONE 3U
#define UART_RX_DONE 4U


typedef struct
{

    uint8_t TxChVal;
    uint8_t RxChVal;

} GPIO_AlternFunc;


static const GPIO_AlternFunc UART_AlternFunc[PORTS_NUMBER]
={

 {0x01,0x01},
 {0x02,0x02},
 {0x01,0x01},
 {0x01,0x01},
 {0x01,0x01},
 {0x01,0x01},
 {0x01,0x01},
 {0x01,0x01}

};


static uint8_t UART_TxLength[UART_GROUPS_NUMBER];
static uint8_t UART_TxCount[UART_GROUPS_NUMBER];
static uint8_t* UART_TxBuffPtr[UART_GROUPS_NUMBER];

static uint8_t UART_RxLength[UART_GROUPS_NUMBER];
static uint8_t UART_RxCount[UART_GROUPS_NUMBER];
static uint8_t* UART_RxBuffPtr[UART_GROUPS_NUMBER];


/* Initialization Function */
UART_ChkType UART_Init (void)
{

    UART_ChkType RetVar;
    uint8_t LoopIndex;
    uint8_t TempVar1;
    uint16_t TempVar2;
    const UART_ConfigType* CfgPtr;
    double BaudRateDiv;

    for (LoopIndex = 0; LoopIndex < UART_GROUPS_NUMBER; LoopIndex++)
    {
        CfgPtr = &UART_ConfigParam[LoopIndex];
        if(((CfgPtr->StopBits)   <=  TwoStopBit)&&
           ((CfgPtr->WordLen)    <=  Data_8)&&
           ((CfgPtr->TxFIFOSize) <=   16)&&
           ((CfgPtr->RxFIFOSize) <=   16)&&
           ((CfgPtr->INT_Enable) <= Enabled)&&
           ((CfgPtr->FIFO_Level) <= FIFO_14)
           )
        {
            /* Configure GPIO port's alternate function as UART*/

            /* Enable UART port clock */
            RCGCUART_REG |= 1 << (CfgPtr->UARTPortID);

            /* Disable the UART port*/
            UARTCTL_REG(CfgPtr->UARTPortID) = 0x00;

            /* BaudRate value */
            BaudRateDiv = SYSCLK /(16* (CfgPtr->BaudRate));

            // The integer part
            UARTIBRD_REG(CfgPtr->UARTPortID) = (uint32_t) BaudRateDiv;

            // The floating part
            BaudRateDiv -= (uint32_t) BaudRateDiv;
            UARTFBRD_REG(CfgPtr->UARTPortID) = (uint32_t)((BaudRateDiv * 64) + 0.5);

            /* Selecting the system clock */
            UARTCC_REG(CfgPtr->UARTPortID) = 0x0;

            /* Configuring the UART Line Control register */
            // Initializing the temporary variable
            TempVar1 = 0x00;
            // Word Length = 8 bits (11 in binary)
            TempVar1 |= (CfgPtr->WordLen) << WLEN_BIT_NO;
            // One stop bit
            TempVar1 |= (CfgPtr->StopBits) << STP2_BIT_NO;
            // Parity
            TempVar1 |= (CfgPtr->Parity) << PEN_BIT_NO;
            // FIFO
            if(((CfgPtr->TxFIFOSize) > 0) ||
               ((CfgPtr->RxFIFOSize) > 0))
            {
            TempVar1 |= (CfgPtr->FIFOEN) << FEN_BIT_NO;
            }
            // Configuring the register
            UARTLCRH_REG(CfgPtr->UARTPortID) = TempVar1;

            //Enabling UART interrupts
            if((CfgPtr->INT_Enable))
            {UARTIM_REG(CfgPtr->UARTPortID)  |= 1 <<RXIM_BIT_NO;     // setting the interrupt mask bit

            }
            if((CfgPtr->FIFOEN)==1)
            {
            TempVar1=(CfgPtr->FIFO_Level) <<RXIFLSEL_BIT_NO;
            UARTIFLS_REG(CfgPtr->UARTPortID)=TempVar1;
            }


            /* Enabling the UART port */
            TempVar2 = 0x00;
            TempVar2 |= 1 << UARTEN_BIT_NO;
            TempVar2 |= 1 << TXE_BIT_NO;
            TempVar2 |= 1 << RXE_BIT_NO;

#if LOOPBACKMODE
            TempVar2 |= 1 << LBE_BIT_NO; // Loopback mode
#endif
            UARTCTL_REG(CfgPtr->UARTPortID) = TempVar2;

            if (CfgPtr->INT_Enable )
            RetVar = UART_OK;

            UART_Driver_State[LoopIndex] = UART_INIT_DONE;
        }
        else
        {
            RetVar = UART_NOK;
        }

    }
    return RetVar;

}




UART_ChkType UART_Tx_Init(const uint8_t* TxBuffPtr, uint8_t TxLen, uint8_t groupID)
{
    UART_ChkType RetVar;
    const UART_ConfigType* CfgPtr;

    if(groupID < UART_GROUPS_NUMBER)
    {
        CfgPtr = &UART_ConfigParam[groupID];

        if (UART_Driver_State[groupID] == UART_INIT_DONE)
        {
            UART_TxLength[groupID] = TxLen;
            UART_TxBuffPtr[groupID] = TxBuffPtr;
            UART_TxCount[groupID] = 0;

            if((UARTFR_REG(CfgPtr->UARTPortID) & (1 << TXFF_BIT_NO)) == 0)
            {
                UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                UART_TxCount[groupID]++;
                UART_Driver_State[groupID] = UART_TX_INIT_DONE;
                RetVar = UART_OK;
            }
            else
            {
                RetVar = UART_NOK;
            }
        }
    }

    else
    {
        RetVar = UART_NOK;

    }

    return RetVar;
}

UART_ChkType UART_Tx (uint8_t groupID)
{

    UART_ChkType RetVar;
    const UART_ConfigType* CfgPtr;

    if(groupID < UART_GROUPS_NUMBER)
    {
        CfgPtr = &UART_ConfigParam[groupID];

        if(UART_Driver_State[groupID] == UART_TX_INIT_DONE)
        {

            if(UART_TxCount[groupID] < UART_TxLength[groupID])
            {
                if((UARTFR_REG(CfgPtr->UARTPortID) & (1 << TXFF_BIT_NO)) == 0)
                {
                UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                UART_TxCount[groupID]++;
                RetVar = UART_OK;
                }
            }
            else
            {
                UART_Driver_State[groupID] = UART_TX_DONE;
                //Call back function
                UART_ConfigParam[groupID].TxDoneCallbackPtr();
                //CfgPtr->TxDoneCallbackPtr();
            }


        }
        else
            {
                RetVar = UART_NOK;
            }
    }
    return RetVar;

}



UART_ChkType UART_Rx_Init (const uint8_t* RxBuffPtr, uint8_t RxLen, uint8_t groupID)
{
    UART_ChkType RetVar;
    const UART_ConfigType* CfgPtr;

    if (groupID < UART_GROUPS_NUMBER)
        {
        CfgPtr = &UART_ConfigParam[groupID];

        if(UART_Driver_State[groupID] == UART_INIT_DONE)
        {
            UART_RxLength[groupID] = RxLen;
            UART_RxCount[groupID] = 0;
            UART_RxBuffPtr[groupID] = RxBuffPtr;


            if((UARTFR_REG(CfgPtr->UARTPortID) & (1 << RXFE_BIT_NO)) == 0)
            {
                *(UART_RxBuffPtr[groupID] + UART_RxCount[groupID]) = UARTDR_REG(CfgPtr->UARTPortID);
                UART_RxCount[groupID]++;
                UART_Driver_State[groupID] = UART_RX_INIT_DONE;
                RetVar = UART_OK;
            }
            else

            {
                RetVar = UART_NOK;
            }

        }

        }
    else
    {
        RetVar=UART_NOK;
    }

    return RetVar;
}


UART_ChkType UART_Rx (uint8_t groupID)
{

    UART_ChkType RetVar;
    const UART_ConfigType* CfgPtr;

    if(groupID < UART_GROUPS_NUMBER)
    {
        CfgPtr = &UART_ConfigParam[groupID];

        if(UART_Driver_State[groupID] == UART_RX_INIT_DONE)
        {
            if(UART_RxCount[groupID] < UART_RxLength[groupID])
            {
                if((UARTFR_REG(CfgPtr->UARTPortID) & (1 << RXFE_BIT_NO)) == 0)
                {
                    *(UART_RxBuffPtr[groupID] + UART_RxCount[groupID]) = UARTDR_REG(CfgPtr->UARTPortID);
                    UART_RxCount[groupID]++;
                    RetVar = UART_OK;
                }
            }
            else
            {
                UART_Driver_State[groupID] = UART_RX_DONE;
                // Callback function
                UART_ConfigParam[groupID].RxDoneCallbackPtr();
                //CfgPtr->RxDoneCallbackPtr();
            }

        }
        else
        {
            RetVar = UART_NOK;
        }

    }
    return RetVar;


}


UART_StatusChkType UART_Get_Status (uint8_t groupID)
{

    UART_StatusChkType RetVar;
    const UART_ConfigType* CfgPtr;

    if(groupID < UART_GROUPS_NUMBER)
    {
        CfgPtr = &UART_ConfigParam[groupID];

        if((UARTFR_REG(CfgPtr->UARTPortID) & (1 << TXFE_BIT_NO) == 1) &&
           (UARTFR_REG(CfgPtr->UARTPortID) & (1 << RXFE_BIT_NO) == 1) &&
           (UART_Driver_State[groupID] != UART_RX_DONE) &&
           (UART_Driver_State[groupID] != UART_TX_DONE))
        {
            RetVar = Idle;
        }

        else
        {

            if((UARTFR_REG(CfgPtr->UARTPortID) & (1 << BUSY_BIT_NO) == 1) &&
               (UARTFR_REG(CfgPtr->UARTPortID) & (1 << TXFE_BIT_NO) == 0))
            {

                if((UART_Driver_State[groupID] != UART_TX_DONE))
                {
                    RetVar = TxInProg;
                }
                else
                {
                    RetVar = TxDone;
                }
            }
            else if ((UARTFR_REG(CfgPtr->UARTPortID) & (1 << RXFE_BIT_NO) == 0))
            {
                if((UART_Driver_State[groupID] != UART_RX_DONE))
                {
                    RetVar = RxInProg;
                }
                else
                {
                    RetVar = RxDone;
                }
            }
        }
    }

    return RetVar;

}


void TxDone_Func (void)
{

    while(1)
    {}
    // TxDone Callback Function

}

void RxDone_Func (void)
{

    while(1)
    {}
    // RxDone Callback Function

}




UART_ChkType UART_Rx_Interrupt (const uint8_t* RxBuffPtr, uint8_t RxLen, uint8_t groupID)
{
    UART_ChkType RetVar;
    const UART_ConfigType* CfgPtr;

    if (groupID < UART_GROUPS_NUMBER)
        {
        CfgPtr = &UART_ConfigParam[groupID];

        if(UART_Driver_State[groupID] == UART_INIT_DONE)
        {
            if(UART_RxCount[groupID] == 0)
            {
                UART_RxLength[groupID] = RxLen;
                UART_RxBuffPtr[groupID] = RxBuffPtr;
            }

            if(UART_RxCount[groupID] <= UART_RxLength[groupID])
            {

                if((UARTFR_REG(CfgPtr->UARTPortID) & (1 << RXFE_BIT_NO)) == 0)

                {
                    *(UART_RxBuffPtr[groupID] + UART_RxCount[groupID]) = UARTDR_REG(CfgPtr->UARTPortID);
                    UART_RxCount[groupID]++;
                    RetVar = UART_OK;
                }
            else

            {
                UART_Driver_State[groupID] = UART_RX_DONE;
            }
            }

        }

        }
    else
    {
        RetVar=UART_NOK;
    }

    return RetVar;
}


void UART0_ISR(void)
{
    uint8_t TempVar = 0;
    UART_Rx_Interrupt(INT_msg_rx_buff,INT_BUFF_LEN,0);
    UARTICR_REG(0)|= (1 << RXIC_BIT_NO);
    TempVar= UARTICR_REG(0);

}

