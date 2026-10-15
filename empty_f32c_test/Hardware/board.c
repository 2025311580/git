#include "ti_msp_dl_config.h"
#include "board.h"

//返回SysTick计数值
uint32_t Systick_getTick(void)
{
	return (SysTick->VAL);
}


//ms阻塞延迟
void delay_ms(uint32_t ms)
{
	/* 一次 delay_us(1000) 等满 1ms，而不是 delay_us(1) 循环一千次。
	 * 后者每次调用都要重新读 SysTick、重新算一遍，-O0 下光是函数调用
	 * 开销就把 1ms 拖成 1.2ms 以上 —— 主循环每 6.667ms 要发一帧目标角度，
	 * 里面夹着好几次 delay_ms(1)，被拖慢就直接吃掉 150Hz 的节拍余量。
	 * delay_us(1000) = 80000 个计数，远小于 0xFFFFFF，不会触发上限钳位。 */
	while (ms--)
	{
		delay_us(1000);
	}
}


void delay_us(uint32_t us)
{
	if( us > SysTickMAX_COUNT/(SysTickFre/1000000) ) us = SysTickMAX_COUNT/(SysTickFre/1000000);
	
	us = us*(SysTickFre/1000000); //单位转换
	
	//用于保存已走过的时间
	uint32_t runningtime = 0;
	
	//获得当前时刻的计数值
	uint32_t InserTick = Systick_getTick();
	
	//用于刷新实时时间
	uint32_t tick = 0;
	
	uint8_t countflag = 0;
	//等待延迟
	while(1)
	{
		tick = Systick_getTick();//刷新当前时刻计数值
		
		if( tick > InserTick ) countflag = 1;//出现溢出轮询,则切换走时的计算方式
		
		if( countflag ) runningtime = InserTick + SysTickMAX_COUNT - tick;
		else runningtime = InserTick - tick;
		
		if( runningtime>=us ) break;
	}

}
