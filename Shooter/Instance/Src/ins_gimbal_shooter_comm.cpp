/* Includes ------------------------------------------------------------------*/
#include "ins_gimbal_shooter_comm.hpp"
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// #TODO ID还没确定好，还是乱设的
hero::GimbalShooterComm unique_gimbal_shooter_comm = hero::GimbalShooterComm(hero::GimbalShooterComm::CodePart::kCodePartInShooter, 0x01, 0x02);
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hero::GimbalShooterComm* CraeteGimbalShooterComm(void) { return &unique_gimbal_shooter_comm; };
/* Private function definitions ----------------------------------------------*/
