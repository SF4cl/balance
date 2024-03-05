#ifndef __RC_H__
#define __RC_H__

#include "main.h"
//#include "can.h"
#include "dma.h"
//#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "stdlib.h"


//typedef struct rc_info *rc_info_t;


/**
  * @brief  remote control information
  */
struct rc_info
{
    /* rocker channel information */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    /* left and right lever information */
    uint8_t sw1;
    uint8_t sw2;
    /* mouse movement and button information */
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;

        uint8_t l;
        uint8_t r;
    } mouse;
    /* keyboard key information */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W : 1;
            uint16_t S : 1;
            uint16_t A : 1;
            uint16_t D : 1;
            uint16_t SHIFT : 1;
            uint16_t CTRL : 1;
            uint16_t Q : 1;
            uint16_t E : 1;
            uint16_t R : 1;
            uint16_t F : 1;
            uint16_t G : 1;
            uint16_t Z : 1;
            uint16_t X : 1;
            uint16_t C : 1;
            uint16_t V : 1;
            uint16_t B : 1;
        } bit;
    } kb;
    int16_t wheel;
};















extern uint8_t rx_buffer[50];   //接收数据的数组



extern void RC_Data_Handle(void);



#endif
