/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef COMMUNICATION_TOOLS_HPP_
#define COMMUNICATION_TOOLS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "can.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
void InitCanFilter(CAN_HandleTypeDef *hcan_init);
void SendCanData(CAN_HandleTypeDef *hcan_tx, uint32_t id, uint8_t tx_data[8]);

#endif /* COMMUNICATION_TOOLS_HPP_ */