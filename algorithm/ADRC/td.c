#include "td.h"

void TD_init(td_type_def *Td,fp32 r,fp32 freq)
{
    if (Td == NULL)
    {
        return;
    }
		Td->freq=freq;
		Td->R=r;
		Td->x1=0;
		Td->x2=0;
		Td->out=0;
}
fp32 xx;
double A;
fp32 TD_calc(td_type_def *Td, fp32 in)
{
		
		A=Td->R*Td->freq;
		
		Td->x2=Td->x2-2*A*Td->x2+A*Td->R*(in-Td->x1);
		Td->x1=Td->x1+Td->x2*Td->freq;
		xx=(in-Td->in)/Td->freq;
		//Td->x2=(in-Td->x1)/Td->freq;
		//Td->x1=in;
		Td->in=in;
		Td->out=Td->x2;
    return Td->out;
}
