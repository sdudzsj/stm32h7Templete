#include "arm_math.h"


#include "my_fft.h"


//extern uint32_t ADC2BUFF[2] = {0};
extern uint16_t myi=0;

extern float32_t input[length*2+1] = {0};//前一个数表示实部，后一个表示虚部
extern float32_t Amp[length]= {0};//存放幅值
extern float32_t rate[length]= {0};//存放频率
extern float32_t Phase[length]= {0};


/*********************************************************************************
************************STM32F407核心开发板******************************
**********************************************************************************
* 文件名称: fft.c                                                             *
* 文件简述：DSP FFT使用                                                           *
* 创建日期：2020.08.21                                                           *
* 版    本：V1.0                                                                 *
* 作    者：近视未看清人心                                                              *
* 说    明：完成FFT信号分析需要用到的函数集，包括求模，幅值、频率、相位
注意：FFT的长度，也就是采样点的个数在fft.h 宏定义中完成* 
**********************************************************************************
*********************************************************************************/	

arm_cfft_radix4_instance_f32 scfft;	//设置FFT参数的结构体变量（基4）

/****************************************************************************
* 名    称: void FFTx4_init(u8 ifftFlag,u8 bitReverseFlag)
* 功    能：FFT初始化函数（基4）
* 入口参数：ifftFlag：于指定是傅里叶变换(0)还是反傅里叶变换(1)
						bitReverseFlag：是否按位取反
                 
* 返回参数：无
* 说    明：
****************************************************************************/

void FFTx4_init(uint8_t ifftFlag,uint8_t bitReverseFlag)
{
	arm_cfft_radix4_init_f32(&scfft,length,ifftFlag,bitReverseFlag);//初始化scfft结构体，设定FFT相关参数
}




/****************************************************************************
* 名    称: void cFFTx4(float * pSrc)
* 功    能：FFT计算函数（基4）
* 入口参数：pSrc：pSrc 传入采集到的输入信
									号数据（实部+虚部形式），同时 FFT 变换后的数据，也按顺序存放在 pSrc 里面                
* 返回参数：无
* 说    明：pSrc的长度必须>=2*length
****************************************************************************/

void cFFTx4(float32_t * pSrc)
{
	arm_cfft_radix4_f32(&scfft,pSrc);	//FFT计算（基4）
}





/****************************************************************************
* 名    称: void cmplxFFTx4(float * pSrc,float * pDst)
* 功    能：对FFT结果求模值（基4）
* 入口参数：pSrc：需要求模的复数指针（实部+虚部形式）
					 pDst：存放模值
					

* 返回参数：无
* 说    明：pSrc的长度必须>=2*length
****************************************************************************/

void cmplxFFTx4(float32_t * pSrc,float32_t * pDst)
{
	arm_cmplx_mag_f32(pSrc,pDst,length);	//计算复数模值

}





/****************************************************************************
* 名    称: void all_result_x4(float * pSrc,float * Amp,float * rate,float * Phase,u32 Fs)
* 功    能：计算各点幅值、频率、相位
* 入口参数：pSrc：原始信号的复数指针（实部+虚部形式）
					 Amp：存放幅值
					 rate：存放各点的频率
					 Phase：存放各点的相位
					 size：采样点的个数

* 返回参数：无
* 说    明：pSrc的长度必须>=2*size
****************************************************************************/
void all_result_x4(float32_t * pSrc,float32_t * Amp,float32_t * rate,float32_t * Phase,uint32_t Fs)
{
	float pDst[length];
	uint32_t i;
	arm_cfft_radix4_f32(&scfft,pSrc);	//FFT计算（基4）
	arm_cmplx_mag_f32(pSrc,pDst,length);	//计算复数模值
	//计算各点幅值、频率、相位
	for(i=0;i<length;i++)
	{
		if(i==0)	Amp[0]=pDst[0]/length;	//第一个点直接除以size(z直流分量)
		else Amp[i]=pDst[i]*2/length;	//其他点除以size/2
		rate[i]=Fs/length*i;//	各点频率
		Phase[i]=atan2(pSrc[2*i+1], pSrc[2*i]); /* atan2求解的结果范围是(-pi, pi], 弧度制 */
	}
	
}
                

/*

FFTx4_init(0,1);//FFT?????(?4)

if(myi==1024){
			all_result_x4(input, Amp, rate, Phase,1024);//????????????
			printf("A:       F;    P\n");
		  HAL_Delay(1);
			for(i=0;i<length;i++){
				//printf("%.2f: %.2f: %.2f\n",Amp[i],rate[i],Phase[i]);//?????????????
				printf("%.2f,%.2f\n",rate[i],Amp[i]);//?????????????
				HAL_Delay(1);
		  }
			
		}

*/



