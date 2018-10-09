#include "delay.h"
#include  <k_api.h>					//ucos 使用	  


static u8  fac_us=0;							//us延时倍乘数			   
//static u16 fac_ms=0;							//ms延时倍乘数,在ucos下,代表每个节拍的ms数
	
	

void SysTick_Handler(void)
{	
	
	
  krhino_intrpt_enter();
  krhino_tick_proc();
  krhino_intrpt_exit();
	
}

			   
//初始化延迟函数
//当使用OS的时候,此函数会初始化OS的时钟节拍
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void delay_init()
{
	u32 reload;
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);	//选择外部时钟  HCLK/8
	fac_us=SystemCoreClock/1000000;				//为系统时钟的1/8  
	reload=SystemCoreClock/1000000;				//每秒钟的计数次数 单位为K	   
	reload*=1000000/RHINO_CONFIG_TICKS_PER_SECOND;	//根据delay_ostickspersec设定溢出时间
												                //reload为24位寄存器,最大值:16777216,在72M下,约合1.86s左右	
	//fac_ms=1000/RHINO_CONFIG_TICKS_PER_SECOND;			//代表OS可以延时的最少单位	   

	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;   	//开启SYSTICK中断
	SysTick->LOAD=reload; 						//每1/delay_ostickspersec秒中断一次	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   	//开启SYSTICK    
}
void delay_us(u32 nus)
{
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload = SysTick->LOAD;
	ticks = nus*fac_us;
	told = SysTick->VAL;
	while(1)
	{
		tnow = SysTick->VAL;
		if(tnow != told)
		{
			if(tnow < told) tcnt += told-tnow;
			else tcnt += reload-tnow+told ;
			told = tnow;
			if(tcnt >= ticks) break;
		}
	};
}
void delay_ms(u16 nums)
{
	u32 i;
	for(i = 0; i < nums;i++)delay_us(1000);
}






























