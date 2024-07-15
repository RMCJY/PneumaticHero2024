/* Includes ------------------------------------------------------------------*/
#include "ins_gimbal_shooter_comm.hpp"
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hero::GimbalShooterComm unique_gimbal_shooter_comm = hero::GimbalShooterComm();
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hero::GimbalShooterComm* CreateGimbalShooterComm(void) { return &unique_gimbal_shooter_comm; };
/* Private function definitions ----------------------------------------------*/
