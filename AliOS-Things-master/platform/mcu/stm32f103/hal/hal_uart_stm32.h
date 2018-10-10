#include <hal/hal.h>
#include "sys.h"
#include "usart.h"	 

#define PORT_UART1 1
#define PORT_UART2 0
#define PORT_UART3 3
#define PORT_UART4 4

#define MAX_BUF_UART_BYTES  1000

typedef struct UART_InitTypeDef
{
	uint32_t       BaudRate;
	uint16_t       WordLength; 
	uint16_t       StopBits ;
	uint16_t       Parity ;
	uint16_t       HardwareFlowControl ;
	uint16_t       Mode ;
}UART_InitTypeDef;
typedef struct UART_HandleTypeDef
{
	USART_TypeDef    *Instance;
	UART_InitTypeDef  Init;
}UART_HandleTypeDef;
