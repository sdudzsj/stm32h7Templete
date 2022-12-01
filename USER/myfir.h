#ifndef _MYFIR_H
#define _MYFIR_H
#include "arm_math.h"

#define TEST_LENGTH_SAMPLES  420    /* 采样点数 */

#define BLOCK_SIZE           42     /* 调用一次arm_fir_f32处理的采样点个数 */

#define NUM_BLOCKS           TEST_LENGTH_SAMPLES/BLOCK_SIZE

#define NUM_TAPS             33     /* 滤波器系数个数 */

extern uint16_t myfiri;

//extern float32_t Input_buffer[TEST_LENGTH_SAMPLES]; /* 采样点 */
extern float32_t INPUTBUFF[TEST_LENGTH_SAMPLES];
extern float32_t OUTPUTBUFF[TEST_LENGTH_SAMPLES];
//extern float32_t Output_buffer[TEST_LENGTH_SAMPLES]; /* 滤波后的输出 */

void arm_fir_f32_lp(float32_t *Input_buffer,float32_t *Output_buffer,uint32_t blockSize);
void arm_fir32_init(uint32_t block_size);


#endif
