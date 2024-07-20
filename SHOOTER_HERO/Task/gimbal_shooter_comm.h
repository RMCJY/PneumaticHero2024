#ifndef __GIMBAL_SHOOTER_COMM_H
#define __GIMBAL_SHOOTER_COMM_H

#include "main.h"

#define SHOOTER_SEND_ID 0x10f

void ShooterGimbalComm(void);
void decodeG2S(uint8_t rx_data[8]);
void encodeS2G(uint8_t tx_data[8]);

#endif