#include <aos/aos.h>
#include <led.h>
#include <stdio.h>

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
int application_start(int argc, char *argv[])
{		
	if(aos_task_new_ext(&Led_Task_t,"LED_TASK",Led_Task,NULL,LED_TASK_STACKSIZE,LED_TASK_PRIORITY)!=0)
		printf("Led task create faild\n");
	if(aos_task_new_ext(&Test_Task_t,"Test_Task",Test_Task,NULL,TEST_TASK_STACKSIZE,TEST_TASK_PRIORITY)!=0)
		printf("Test task create faild\n");
	if(aos_task_new_ext(&Test1_Task_t,"Test1_Task",Test1_Task,NULL,TEST1_TASK_STACKSIZE,TEST1_TASK_PRIORITY)!=0)
		printf("Test task create faild\n");
	
 }

