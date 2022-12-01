#include "myh7fft.h"
#include "arm_math.h"


uint16_t myi=0;
uint8_t ifftFlag = 0;

float32_t input[length] = {0};//前一个数表示实部，后一个表示虚部
float32_t Amp[length]= {0};//存放幅值
float32_t rate[length]= {0};//存放频率
float32_t Phase[length]= {0};






/****************************************************************************
* 名    称: void arm_rfft_f32_app(float * pSrc,float * Amp,float * rate,float * Phase,u32 Fs)
* 功    能：计算各点幅值、频率、相位
* 入口参数：pSrc：原始信号的指针
					 Amp：存放幅值
					 rate：存放各点的频率
					 Phase：存放各点的相位
					 size：采样点的个数

* 返回参数：无
* 说    明：  
* 调用方式：arm_rfft_f32_app(input, Amp, rate, Phase,(uint32_t)(200000000/200/100));
****************************************************************************/
void arm_rfft_f32_app(float32_t * pSrc,float32_t * Amp,float32_t * rate,float32_t * Phase,uint32_t Fs){
	float32_t pDst[2*length];
	uint32_t i;
	arm_rfft_fast_instance_f32 S;	//设置FFT参数的结构体变量（基4）
	//初始化
	arm_rfft_fast_init_f32(&S,length);
	//1024点实序列快速FFT
  arm_rfft_fast_f32(&S,&input[0],&pDst[0],ifftFlag);
	//计算各点幅值、频率、相位
	for(i=0;i<length;i++)
	{
		Amp[i]=pDst[2*i]/length;
		rate[i]=Fs/length*i;//	各点频率
		//Phase[i]=atan2(pSrc[2*i+1], pSrc[2*i]); /* atan2求解的结果范围是(-pi, pi], 弧度制 */
	}
}

