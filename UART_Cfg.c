#include "UART.h"

const UART_ConfigType UART_ConfigParam[UART_GROUPS_NUMBER] =
{
 {
   0x00,
   1<<1,
   1<<0,
   9600,
   OneStopBit,
   Data_8,
   Disabled,
   Enabled,
   16,
   16,
   &TxDone_Func,
   &RxDone_Func,
   Enabled,
   FIFO_2

  }
};
