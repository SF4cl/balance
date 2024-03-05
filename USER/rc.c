#include "rc.h"




struct rc_info rc;
volatile uint8_t buff[50];

void RC_Data_Handle(void)
{
	uint8_t i;
	
	for(i=1;i<=18;i++)
	{
		buff[i] = rx_buffer[i];
	}
	
	
	
	
	rc.ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
	rc.ch1 -= 1024;
	rc.ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
	rc.ch2 -= 1024;
	rc.ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
	rc.ch3 -= 1024;
	rc.ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
	rc.ch4 -= 1024;

	/* prevent remote control zero deviation */
	if (rc.ch1 <= 5 && rc.ch1 >= -5)
	{
			rc.ch1 = 0;
	}
	if (rc.ch2 <= 5 && rc.ch2 >= -5)
	{
			rc.ch2 = 0;
	}
	if (rc.ch3 <= 5 && rc.ch3 >= -5)
	{
			rc.ch3 = 0;
	}
	if (rc.ch4 <= 5 && rc.ch4 >= -5)
	{
			rc.ch4 = 0;
	}

	rc.sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
	rc.sw2 = (buff[5] >> 4) & 0x0003;

	if ((abs(rc.ch1) > 660) || \
					(abs(rc.ch2) > 660) || \
					(abs(rc.ch3) > 660) || \
					(abs(rc.ch4) > 660))
	{
			memset(&rc, 0, sizeof(struct rc_info));
			return ;
	}

	rc.mouse.x = buff[6] | (buff[7] << 8); // x axis
	rc.mouse.y = buff[8] | (buff[9] << 8);
	rc.mouse.z = buff[10] | (buff[11] << 8);

	rc.mouse.l = buff[12];
	rc.mouse.r = buff[13];

	rc.kb.key_code = buff[14] | buff[15] << 8; // key borad code
	rc.wheel = (buff[16] | buff[17] << 8) - 1024;	
	

}







