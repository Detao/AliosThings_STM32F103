#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include <aos/aos.h>

void aos_heap_set(void);
#define LED_TASK_STACKSIZE 256
#define LED_TASK_PRIORITY 20
aos_task_t Led_Task_t;

#define TEST_TASK_STACKSIZE 256
#define TEST_TASK_PRIORITY 20
aos_task_t Test_Task_t;

#define TEST1_TASK_STACKSIZE 256
#define TEST1_TASK_PRIORITY 20
aos_task_t Test1_Task_t;
void Test_Task(void* a)
{
	int num=0;
	printf("kernel version is %s\n",aos_version_get());
	while(1)
	{
		num++;
		printf("task run num is %d\n",num);
		aos_msleep(1000);
	}
}	
void Test1_Task(void* a)
{
	int num=0;
	printf("kernel version is %s\n",aos_version_get());
	while(1)
	{
		num++;
		printf("task1 run num is %d\n",num);
		aos_msleep(1000);
	}
}	
void Led_Task(void* a)
{
	while(1)
	{
		LED0 =~LED0;
		aos_msleep(1000);
	}
}
 int main(void)
 {		
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 //串口初始化为115200
 	LED_Init();			     //LED端口初始化
	KEY_Init();          //初始化与按键连接的硬件接口
  aos_heap_set();
	aos_init();
	
	if(aos_task_new_ext(&Led_Task_t,"LED_TASK",Led_Task,NULL,LED_TASK_STACKSIZE,LED_TASK_PRIORITY)!=0)
		printf("Led task create faild\n");
	if(aos_task_new_ext(&Test_Task_t,"Test_Task",Test_Task,NULL,TEST_TASK_STACKSIZE,TEST_TASK_PRIORITY)!=0)
		printf("Test task create faild\n");
	if(aos_task_new_ext(&Test1_Task_t,"Test1_Task",Test1_Task,NULL,TEST1_TASK_STACKSIZE,TEST1_TASK_PRIORITY)!=0)
		printf("Test task create faild\n");
	aos_start();
	
 }

