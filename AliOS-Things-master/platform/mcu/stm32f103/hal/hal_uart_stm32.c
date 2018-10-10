#include "hal_uart_stm32.h"
#include <k_api.h>

void uart1_MspInit(void);
void HAl_uart_init(UART_HandleTypeDef *uart);
int32_t uart1_init(uart_dev_t *uart);
UART_HandleTypeDef * uart_get_handle(uint8_t port);


/*hadle for uart*/
UART_HandleTypeDef uart1_handle;

/*bufferQueue for uart*/
kbuf_queue_t g_buf_queue_uart1;
char g_buf_uart1[MAX_BUF_UART_BYTES];

/*function used to transform hal para to stm32f103 para*/

static int32_t uart_dataWidth_transform(hal_uart_data_width_t data_width_hal, 
               uint16_t *data_width_stm32l4);
static int32_t uart_parity_transform(hal_uart_parity_t parity_hal, 
               uint16_t *parity_stm32l4);
static int32_t uart_stop_bits_transform(hal_uart_stop_bits_t stop_bits_hal, 
               uint16_t *stop_bits_stm32l4);
static int32_t uart_flow_control_transform(hal_uart_flow_control_t flow_control_hal, 
               uint16_t *flow_control_stm32l4);
static int32_t uart_mode_transform(hal_uart_mode_t mode_hal, uint16_t *mode_stm32l4);

//void uart1_init(uart_dev_t *uart);

int32_t hal_uart_recv_II(uart_dev_t *uart, void *data, uint32_t expect_size,
                         uint32_t *recv_size, uint32_t timeout)
{
	return 0;
}
int32_t hal_uart_recv(uart_dev_t *uart, void *data, uint32_t expect_size, uint32_t timeout)
{
	return 0;
}
int32_t hal_uart_send(uart_dev_t *uart, const void *data, uint32_t size, uint32_t timeout)
{
	  UART_HandleTypeDef *handle = NULL;
	  int i=0;
    int ret = -1;

    if ((uart == NULL) || (data == NULL)) {
        return -1;
    }

    handle = uart_get_handle(uart->port);
    if (handle == NULL) {
        return -1;
    }

    if((uart != NULL) && (data != NULL)) {
			uint8_t *dataSend=(uint8_t *)data;
			while(i < size)
			{
				u8 ch = dataSend[i++];
        while((handle->Instance->SR&0X40)==0);  
        handle->Instance->DR = (u8) ch;
				
			}
    }

    return ret;
}
int32_t hal_uart_init(uart_dev_t *uart)
{
    int32_t ret = -1;

    if (uart == NULL) {
		    return -1;
		}

    switch (uart->port) {
        case PORT_UART1:
      	    uart->priv = &uart1_handle;				
             ret = uart1_init(uart);
				    break;
        case PORT_UART2:
//      	    uart->priv = &uart2_handle;				
//            ret = uart2_init(uart);
				    break;
				/* if other uart exist add init code here */

			  default:
					break;
		}

		return ret;
}
int32_t uart1_init(uart_dev_t *uart)
{
	int32_t ret = -1;
	
	uart1_handle.Instance        = USART1;
	uart1_handle.Init.BaudRate   = uart->config.baud_rate;
	ret = uart_dataWidth_transform(uart->config.data_width, &uart1_handle.Init.WordLength);
  ret |= uart_parity_transform(uart->config.parity, &uart1_handle.Init.Parity);
  ret |= uart_stop_bits_transform(uart->config.stop_bits, &uart1_handle.Init.StopBits);
  ret |= uart_flow_control_transform(uart->config.flow_control, &uart1_handle.Init.HardwareFlowControl);
  ret |= uart_mode_transform(uart->config.mode, &uart1_handle.Init.Mode);

  if(ret != 0){
		
     return -1;
    }
	uart1_MspInit();
	//uart_init(115200);
	
	HAl_uart_init(&uart1_handle);
	ret = krhino_buf_queue_create(&g_buf_queue_uart1, "buf_queue_uart",
          g_buf_uart1, MAX_BUF_UART_BYTES, 1);
	return ret;
}
void uart1_MspInit()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
}
void HAl_uart_init(UART_HandleTypeDef *uart){
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = uart->Init.BaudRate ;//串口波特率
	USART_InitStructure.USART_WordLength = uart->Init.WordLength;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = uart->Init.StopBits;//一个停止位
	USART_InitStructure.USART_Parity = uart->Init.Parity;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = uart->Init.HardwareFlowControl;//无硬件数据流控制
	USART_InitStructure.USART_Mode = uart->Init.Mode;	//收发模式

  USART_Init(uart->Instance, &USART_InitStructure); //初始化串口1
  USART_ITConfig(uart->Instance, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(uart->Instance, ENABLE);                    //使能串口1 
}
void uart_init(u32 bound){
  
	USART_InitTypeDef USART_InitStructure;
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 

}

int32_t uart_dataWidth_transform(hal_uart_data_width_t data_width_hal,
        uint16_t *data_width_stm32f103)
{
    uint32_t data_width = 0;
    int32_t	ret = 0;

    if(data_width_hal == DATA_WIDTH_7BIT)
    {
        data_width = USART_WordLength_8b;
    }
    else if(data_width_hal == DATA_WIDTH_8BIT)
    {
        data_width = USART_WordLength_8b;
    }
    else if(data_width_hal == DATA_WIDTH_9BIT)
    {
        data_width = USART_WordLength_9b;
    }
    else
    {
        ret = -1;
    }

    if(ret == 0)
    {
        *data_width_stm32f103 = data_width;
    }

    return ret;
}

int32_t uart_parity_transform(hal_uart_parity_t parity_hal,
        uint16_t *parity_stm32f103)
{
    uint32_t parity = 0;
    int32_t	ret = 0;

    if(parity_hal == NO_PARITY)
    {
        parity = USART_Parity_No;
    }
    else if(parity_hal == ODD_PARITY)
    {
        parity = USART_Parity_Even;
    }
    else if(parity_hal == EVEN_PARITY)
    {
        parity = USART_Parity_Odd;
    }
    else
    {
        ret = -1;
    }

    if(ret == 0)
    {
        *parity_stm32f103 = parity;
    }

    return ret;
}

int32_t uart_stop_bits_transform(hal_uart_stop_bits_t stop_bits_hal,
        uint16_t *stop_bits_stm32f103)
{
    uint32_t stop_bits = 0;
    int32_t	ret = 0;

    if(stop_bits_hal == STOP_BITS_1)
    {
        stop_bits = USART_StopBits_1;
    }
    else if(stop_bits_hal == STOP_BITS_2)
    {
        stop_bits = USART_StopBits_2;
    }
    else
    {
        ret = -1;
    }

    if(ret == 0)
    {
        *stop_bits_stm32f103 = stop_bits;
    }

    return ret;
}

int32_t uart_flow_control_transform(hal_uart_flow_control_t flow_control_hal,
        uint16_t *flow_control_stm32f103)
{
    uint16_t flow_control = 0;
    int32_t	ret = 0;

    if(flow_control_hal == FLOW_CONTROL_DISABLED)
    {
        flow_control = USART_HardwareFlowControl_None;
    }
    else if(flow_control_hal == FLOW_CONTROL_CTS)
    {
        flow_control = USART_HardwareFlowControl_CTS;
    }
    else if(flow_control_hal == FLOW_CONTROL_RTS)
    {
        flow_control = USART_HardwareFlowControl_RTS;
    }
    else if(flow_control_hal == FLOW_CONTROL_CTS_RTS)
    {
    	flow_control = FLOW_CONTROL_CTS_RTS;
    }
    else
    {
    	ret = -1;
    }

    if(ret == 0)
    {
    	*flow_control_stm32f103 = flow_control;
    }

    return ret;
}

int32_t uart_mode_transform(hal_uart_mode_t mode_hal, uint16_t *mode_stm32f103)
{
    uint16_t mode = 0;
    int32_t	ret = 0;

    if(mode_hal == MODE_TX)
    {
        mode = USART_Mode_Tx;
    }
    else if(mode_hal == MODE_RX)
    {
        mode = USART_Mode_Rx;
    }
    else if(mode_hal == MODE_TX_RX)
    {
        mode = USART_Mode_Tx|USART_Mode_Rx;
    } 
    else
    {
        ret = -1;
    }

    if(ret == 0)
    {
        *mode_stm32f103 = mode;
    }

    return ret;
}
UART_HandleTypeDef * uart_get_handle(uint8_t port)
{
    UART_HandleTypeDef *handle = NULL;

    if (port == PORT_UART1) {
        handle = &uart1_handle;
    } 
		/*else if (port == PORT_UART2) {
        //handle = &uart2_handle;
    } */
		else {
        handle = NULL;
    }

    return handle;
}

