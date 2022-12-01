#ifndef _MY_FFT_H
#define _MY_FFT_H
#include "arm_math.h"

#define length 1024 //FFT长度（16/64/256/1024/4096）,即采样点的个数


extern float32_t input[length];//前一个数表示实部，后一个表示虚部
extern float32_t Amp[length];//存放幅值
extern float32_t rate[length];//存放频率
extern float32_t Phase[length];
//extern uint32_t ADC2BUFF[2];
extern uint16_t myi;

void FFTx4_init(uint8_t ifftFlag,uint8_t bitReverseFlag);//FFT初始化函数（基4）
void cFFTx4(float32_t * pSrc);//FFT计算函数（基4）
void cmplxFFTx4(float32_t * pSrc,float32_t * pDst);//对FFT结果求模值（基4）
void all_result_x4(float32_t * pSrc,float32_t * Amp,float32_t * rate,float32_t * Phase,uint32_t Fs);//计算各点幅值、频率、相位


#endif
