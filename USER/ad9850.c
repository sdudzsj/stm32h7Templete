#include "ad9850.h"
#include "main.h"



void AD9850_GPIO_Config(void)
{
//	/*定义一个GPIO_InitTypeDef类型的结构体*/
//	GPIO_InitTypeDef GPIO_InitStructure;

//	/*开启驱动板上引脚相关的GPIO外设时钟*/
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB,ENABLE);
//	
//	/*配置CLK*/
//	GPIO_InitStructure.GPIO_Pin = CLK_GPIO_PIN;	         //选择要控制的GPIO引脚
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //设置引脚模式为推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //设置速率为50MHz
//	GPIO_Init(CLK_GPIO_PORT, &GPIO_InitStructure);	     //调用库函数，初始化GPIO
//	
//	/*配置FU_UD*/
//	GPIO_InitStructure.GPIO_Pin = FU_UD_GPIO_PIN;
//	GPIO_Init(FU_UD_GPIO_PORT, &GPIO_InitStructure);
//	
//	/*配置RES*/
//	GPIO_InitStructure.GPIO_Pin = RES_GPIO_PIN;
//	GPIO_Init(RES_GPIO_PORT, &GPIO_InitStructure);
//	
//	/*配置DATA*/
//	GPIO_InitStructure.GPIO_Pin = DATA_GPIO_PIN;
//	GPIO_Init(DATA_GPIO_PORT, &GPIO_InitStructure);
//	
//	CLK_LOW;
//	FU_UD_LOW;
//	RES_LOW;
//	GPIO_ResetBits(DATA_GPIO_PORT, DATA_GPIO_PIN);
}


/*---------------------------------------------------*/
// AD9850串行复位函数
/*---------------------------------------------------*/
void AD9850_Serial_Reset(void)
{
	CLK_LOW;
	FU_UD_LOW;
	
	RES_LOW;
	DELAY;
	RES_HIGH;
	DELAY;
	RES_LOW;
	
	CLK_LOW;
	DELAY;
	CLK_HIGH;
	DELAY;
	CLK_LOW;
	
	FU_UD_LOW;
	DELAY;
	FU_UD_HIGH;
	DELAY;
	FU_UD_LOW;
}


/*---------------------------------------------------
@brief	AD9850串行写入函数
@param	frequence 频率(Hz)
@param	phase 相位(0x00~0x1F对应0~360°可调 11.25°步进)
@note		
@Sample	AD9850_Serial_Write(1000, 0x04); //频率1k 初始相位45°
---------------------------------------------------*/
void AD9850_Serial_Write(double frequence, u8 phase)
{
	u8 i;
	u32 fre;
	
	//计算频率
	frequence = frequence * CLOCK_NUM; 
	fre = frequence;
	
	//频率控制字写入
	for(i=0;i<32;i++)
	{
		if((fre>>i) & 0x01)
			DATA_HIGH;
		else
			DATA_LOW;
		//GPIO_WriteBit(DATA_GPIO_PORT, DATA_GPIO_PIN, (BitAction)((fre>>i) & 0x01));
		CLK_HIGH;
		DELAY;
		CLK_LOW;
		DELAY;
	}
	
	//相位控制字写入
	for(i=0;i<8;i++)
	{
		if((phase>>i) & 0x01)
			DATA_HIGH;
		else
			DATA_LOW;
		//GPIO_WriteBit(DATA_GPIO_PORT, DATA_GPIO_PIN, (BitAction)((phase>>i) & 0x01));
		CLK_HIGH;
		DELAY;
		CLK_LOW;
		DELAY;
	}
	
	//使能
	FU_UD_HIGH;
	DELAY;
	FU_UD_LOW;
}









