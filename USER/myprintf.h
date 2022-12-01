#ifndef _MY_PRINTF_H
#define _MY_PRINTF_H

#include "main.h"
#include "stdio.h"

extern uint8_t ch;
extern uint8_t ch_r;

int fputc(int c, FILE * f);
int fgetc(FILE * F);

#endif
