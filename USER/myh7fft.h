#ifndef _MYH7FFT_H
#define _MYH7FFT_H

#include "arm_math.h"


#define length 1024

extern float32_t input[length];//前一个数表示实部，后一个表示虚部
extern float32_t Amp[length];//存放幅值
extern float32_t rate[length];//存放频率
extern float32_t Phase[length];


extern uint16_t myi;



void arm_rfft_f32_app(float32_t * pSrc,float32_t * Amp,float32_t * rate,float32_t * Phase,uint32_t Fs);



#endif /*_MYH7FFT_H*/


