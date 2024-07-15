/** 
 *******************************************************************************
 * @file      : communication_tools.hpp
 * @brief     : 
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef COMMUNICATION_TOOLS_HPP_
#define COMMUNICATION_TOOLS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "usart.h"
#include "iwdg.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
void InitCanFilter(CAN_HandleTypeDef *hcan);
void SendCanData(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t tx_data[8]);

#endif /* COMMUNICATION_TOOLS_HPP_ */
