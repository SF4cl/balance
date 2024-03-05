/*
* 这个任务用于向云台发送信息
* 本任务处于最高优先级，是实时性的
* 问题：目前发送信息放在中断里，会导致FIFO区撑满，从而发送失败
* 解决思路：建立两个FIFO缓存区，分别对应高低优先级，高优先级的数据塞进高优先级FIFO；
*			每次优先发送直到高优先级的FIFO，直至其发送完毕才发送低优先级FIFO
* 实现部分：当数据FIFO出现错误时，实现亮灯，但是不能检测是否成功发送
* 修改：1.进行数据同步，需要注意的是，一共有两个FIFO缓存区，以及一个裁判系统数据缓存区。
*			目前思路为接收FIFO是一直在接数的，当接收任务运行时，接收FIFO清空，同时将数据存入发送FIFO、
*			以及裁判系统数据缓存区，此时接收任务挂起，发送任务运行。发送FIFO清空时，发送任务挂起，接收任务运行。
*			中间接收FIFO始终在运行，可对数据最大化利用。
*/
#include "cmsis_os.h"
#include "CanSendThread.h"
#include "tim.h"
#include "can.h"
#include "RefereeBehaviour.h"


uint8_t DataBuffer[11];
extern osThreadId CanSendHandle;
extern osThreadId RefereeHandle;

static CAN_TxHeaderTypeDef can_tx_message;

static void Can_Send(CAN_HandleTypeDef *hcan,uint32_t id,uint8_t lenth,uint8_t *buffer);
void CanSendTask(void const * argument)
{
	uint8_t SendLenth;
	uint32_t SendId;
	while(1)
	{
		if(Pop(SendBuffer,(SendBuffer+1),DataBuffer) == 1)
		{
			SendId = (uint32_t)(DataBuffer[0]<<8|DataBuffer[1]);

			SendLenth = DataBuffer[2];

			Can_Send(&hcan1,SendId,SendLenth,&DataBuffer[3]);
		}

		osDelay(1);
	}
}

static void Can_Send(CAN_HandleTypeDef *hcan,uint32_t id,uint8_t lenth,uint8_t *buffer)
{
	uint32_t send_mail_box;
    can_tx_message.StdId = id;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = lenth;
	
    HAL_CAN_AddTxMessage(hcan, &can_tx_message, buffer, &send_mail_box);
}

uint32_t CMSCounter = 0;
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  if (htim->Instance == TIM7) {
//    HAL_IncTick();
//  }
//  else if (htim->Instance == TIM3) {
//    CMSCounter++;
//  }
//  
//}