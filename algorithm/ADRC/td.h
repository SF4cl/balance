#ifndef TD_H
#define TD_H
#include "struct_typedef.h"
#include "main.h"

typedef struct
{

    fp32 R,freq;
    fp32 x1,x2;
    fp32 in,out;

} td_type_def;

extern void TD_init(td_type_def *Td,fp32 r,fp32 freq);
extern fp32 TD_calc(td_type_def *Td, fp32 in);

#endif
