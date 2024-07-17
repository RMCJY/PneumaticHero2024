#include "HW_CAN.h"
#include "can.h"
#include "shooter.h"
#include "gimbal_shooter_comm.h"

// TODO：修改为裁判系统捷报的那种形式，反馈信息需要更加完整
void ShooterGimbalComm(void)
{
    uint8_t tx_data[8] = {0};
    encodeS2G(tx_data);
    CAN_Send_Msg(&hcan, tx_data, SHOOTER_SEND_ID , 8);
}

void decodeG2S(uint8_t rx_data[8])
{
    shooter.shooter_signal = rx_data[0];
    shooter.is_rfr_shooter_power_on = rx_data[1];
    shooter.rfr_bullet_speed = (float)(rx_data[2] << 8 | rx_data[3]) / 100.0;
}

void encodeS2G(uint8_t tx_data[8])
{   
    // 部分状态需要显示在UI上
    tx_data[0] = shooter.is_shooter_ready;
    tx_data[1] = shooter.bullet_feed_note;
    tx_data[2] = shooter.shooter_clear_note;
    tx_data[3] = shooter.work_state;
    tx_data[4] = shooter.is_air_bottle_ready;
    tx_data[5] = shooter.is_airpre_ready;
}