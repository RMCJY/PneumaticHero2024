/*
 * @Author: RMCJY 1409947012@qq.com
 * @Date: 2024-05-25 21:13:31
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-05-25 21:55:04
 * @FilePath: \Shooter\Hero-Shooter-Components\Src\communication_tools.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/* Includes ------------------------------------------------------------------*/
#include "communication_tools.hpp"
#include "stm32f1xx_hal_can.h"
#include "main.h"
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

CAN_TxHeaderTypeDef can_tx_header_debug = {0};  ///< CAN 发送数据帧头定义 【dubug用】
uint32_t can_tx_mail_box_debug = 0;             ///< CAN 发送数据帧邮箱 【dubug用】
size_t can_tx_error_times = 0;                  ///< CAN 发送数据帧错误次数 【dubug用】

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

/**
* @note   STM32F103RET6只有一路CAN，这边的初始化函数做相应更改
**/
void InitCanFilter(CAN_HandleTypeDef *hcan_init)
{
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
};

void SendCanData(CAN_HandleTypeDef *hcan_tx, uint32_t id, uint8_t tx_data[8])
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint32_t mail_box = 0;
  tx_header.StdId = id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;

  can_tx_header_debug = tx_header;
  can_tx_mail_box_debug = mail_box;

  if (HAL_CAN_AddTxMessage(hcan_tx, &tx_header, tx_data, &mail_box) != HAL_OK) {
    can_tx_error_times++;
  }
};
/* Private function definitions ----------------------------------------------*/
