/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-01-27 14:16:18
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-06-09 20:47:38
 * @FilePath: \LOWER_FINAL\Task\HW_CAN.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

/* ------------------------------ Include ------------------------------ */

#include "HW_CAN.h"
#include "shooter.h"

/* ------------------------------ Function Definition ------------------------------ */
/**
* @brief  can filter的初始化，直接抄的官方代码
* @param  can的结构体
* @retval none
* @note   必须在主函数中初始化
**/
void CanFilter_Init(CAN_HandleTypeDef* hcan_init) {
    CAN_FilterTypeDef canfilter;

    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;

    //filtrate any ID you want here
    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;  
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;

    canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
    canfilter.FilterActivation = ENABLE;
    canfilter.SlaveStartFilterBank = 14;

    if(hcan_init == &hcan)
    {
        canfilter.FilterBank = 0;
    }
    if (HAL_CAN_ConfigFilter(hcan_init, &canfilter) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
* @brief  can中断的回调函数,解析全部在这里
* @param  can的结构体
* @retval none
* @note   收发数据的长度都只需要1byte（DLC=1）
**/
CAN_RxHeaderTypeDef  RxMessageHeader = {0}; 
static int times = 0;
uint8_t RxData[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan_rx)
{
	uint8_t rawData[8];
    if(hcan_rx == &hcan)
    {
		if (HAL_CAN_GetRxMessage(hcan_rx, CAN_RX_FIFO0, &RxMessageHeader, rawData) == HAL_OK)		// 获得接收到的数据头和数据
		{
			if(RxMessageHeader.StdId == MASTER_BASEADDR)      //判断数据帧头ID，这里表示上位机
            {
                // RxData[0] = rawData[0];
                // RxData[1] = rawData[1];
                // RxData[2] = rawData[2];
                // // 第一位为0、1
                // // 1发射
                // // 0不发射
                // if(RxData[0] == 1)
                // {
                //     if(shooter.shooter_command == 2)
                //     {
                //         shooter.shooter_command = 3;
                //     }
                //     else
                //     {
                //         shooter.shooter_command = 1;
                //     }
                // }
                if(shooter.is_shooter_ready == SHOOTER_IDLE)
                {
                    /*测试使用，将23/25分开来*/
                    shooter.shooter_command = SHOOTER_COMMAND_ACTIVE;
                    // shooter.shooter_command = rawData[0];
                    HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,RESET);
                }
                else if(shooter.shooter_command == SHOOTER_COMMAND_ACTIVE)
                {
                    /*测试使用，将23/25分开来*/
                    shooter.shooter_command = SHOOTER_COMMAND_NEXT;
                }
			}
            if(RxMessageHeader.StdId == SPEED_ADRR)
            {
                shooter.bullet_speed = (rawData[0] * 256 + rawData[1]) / 1000;
            }
        }
    }
    HAL_CAN_ActivateNotification(hcan_rx, CAN_IT_RX_FIFO0_MSG_PENDING);    // 再次使能FIFO0接收中断    
}

/**
* @brief  向can总线发送数据，抄官方的
* @param  can的结构体
* @param  发送的内容
* @param  发送ID
* @param  数据长度
* @retval none
* @note   
**/
CAN_TxHeaderTypeDef  TxMessageHeader= {0};
uint32_t pTxMailbox = 0;             //传入HAL_CAN_AddTxMessage()提供存储空间
uint8_t can_flag = 0;
void CAN_Send_Msg(CAN_HandleTypeDef* hcan, uint8_t* msg, uint32_t id, uint8_t len)
{
	TxMessageHeader.StdId = id;
	TxMessageHeader.IDE = CAN_ID_STD;
	TxMessageHeader.RTR = CAN_RTR_DATA;
	TxMessageHeader.DLC = len;
	if(HAL_CAN_AddTxMessage(hcan,&TxMessageHeader,msg,&pTxMailbox)!=HAL_OK)
	{
        can_flag++;
    }
}