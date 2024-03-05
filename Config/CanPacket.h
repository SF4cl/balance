#ifndef __CANPACKET_H__
#define __CANPACKET_H__

#include "struct_typedef.h"
#include "string.h"
#include "struct_typedef.h"


typedef enum
{
	DefaultAimStatusAndTargetId = 0x106,
	SentryAimStatusAndTargetId = 0x107,
	DefaulPTZRequestAndStatusId = 0x110,
	SentryPTZRequestAndStatusId = 0x111,
}CanReceive_e;

typedef __PACKED_STRUCT
{
	uint8_t AimStatus;
	uint8_t AimTarget;
}Aim_t;

typedef __PACKED_STRUCT
{
	uint8_t AimTargetRequest;
	int16_t FBSpeed;
	int16_t LRSpeed;
	uint8_t ChassisStatueRequest;
	uint8_t PTZStatusInformation;
}PTZ_t;






typedef __PACKED_STRUCT
{
	uint8_t robot_id;
	uint8_t power_output;
	uint16_t remain_HP;
	uint16_t max_HP;
} robot_information_t;

typedef __PACKED_STRUCT
{
	uint16_t hero_remain_HP;
	uint16_t infantry3_remain_HP;
	uint16_t infantry4_remain_HP;
	uint16_t infantry5_remain_HP;
} enemy_information_t;

typedef __PACKED_STRUCT
{
	uint8_t game_status;
	uint16_t end_time;
} send_game_status_t;


extern uint8_t *send_power_heat_data(void);
extern uint8_t *send_bullet_speed(void);
extern uint8_t *send_bullet_limit(void);
extern uint8_t *send_power_limit(void);
extern uint8_t *send_robot_information(void);
extern uint8_t *send_enemy_information(void);
extern uint8_t *send_game_status(void);

//这两个结构体的使用，参见《CAN总线数据内容及数据格式规定》
extern Aim_t Aim;
extern PTZ_t PTZ;


#endif