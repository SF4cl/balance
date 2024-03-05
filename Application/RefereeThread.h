#ifndef __REFEREETASK_H
#define __REFEREETASK_H
#include "main.h"

#include "fifo.h"
#include "RefereeBehaviour.h"
#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024
	
void RefereeTask(void const * argument);	
extern	uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
extern fifo_s_t referee_fifo;
extern uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
extern unpack_data_t referee_unpack_obj;

void referee_unpack_fifo_data(void);

#endif