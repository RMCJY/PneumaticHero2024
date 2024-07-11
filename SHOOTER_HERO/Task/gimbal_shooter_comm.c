#include "HW_CAN.h"
#include "can.h"
#include "shooter.h"
#include "gimbal_shooter_comm.h"

void ShooterGimbalComm(void)
{
    uint8_t tx_data[8] = {0};
    tx_data[0] = shooter.bullet_note;
    tx_data[1] = shooter.is_shooter_ready;
    tx_data[2] = shooter.is_air_bottle_ready;
    CAN_Send_Msg(&hcan, tx_data, SHOOTER_SEND_ID , 8);
}