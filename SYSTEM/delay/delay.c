#include "delay.h"
#include  <k_api.h>					//ucos ʹ��	  


static u8  fac_us=0;							//us��ʱ������			   
//static u16 fac_ms=0;							//ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��
	
	

void SysTick_Handler(void)
{	
	
	
  krhino_intrpt_enter();
  krhino_tick_proc();
  krhino_intrpt_exit();
	
}

			   
//��ʼ���ӳٺ���
//��ʹ��OS��ʱ��,�˺������ʼ��OS��ʱ�ӽ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
void delay_init()
{
	u32 reload;
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);	//ѡ���ⲿʱ��  HCLK/8
	fac_us=SystemCoreClock/1000000;				//Ϊϵͳʱ�ӵ�1/8  
	reload=SystemCoreClock/1000000;				//ÿ���ӵļ������� ��λΪK	   
	reload*=1000000/RHINO_CONFIG_TICKS_PER_SECOND;	//����delay_ostickspersec�趨���ʱ��
												                //reloadΪ24λ�Ĵ���,���ֵ:16777216,��72M��,Լ��1.86s����	
	//fac_ms=1000/RHINO_CONFIG_TICKS_PER_SECOND;			//����OS������ʱ�����ٵ�λ	   

	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;   	//����SYSTICK�ж�
	SysTick->LOAD=reload; 						//ÿ1/delay_ostickspersec���ж�һ��	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   	//����SYSTICK    
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






























