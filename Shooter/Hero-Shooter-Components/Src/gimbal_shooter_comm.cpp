/*
 * @Author: RMCJY 1409947012@qq.com
 * @Date: 2024-05-25 21:14:03
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-06-30 16:02:21
 * @FilePath: \Shooter\Hero-Shooter-Components\Src\gimbal_shooter_comm.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/* Includes ------------------------------------------------------------------*/
#include "gimbal_shooter_comm.hpp"
#include "pid.hpp"

#include <cstring>

namespace hero
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/

struct __attribute__((packed)) Gimbal2ShooterPkg{
    // uint8_t gimbal_work_state : 2;
    uint8_t main_board_gimbal_work_state : 2;
    uint8_t shooter_command : 1;
};

struct __attribute__((packed)) Shooter2GimbalPkg{
    uint8_t main_board_shooter_work_state : 2;
    uint8_t is_shooter_ready : 1;
    uint8_t is_shooter_pill_ready : 1;
    uint8_t is_airpre_ready : 1;
    uint8_t is_air_bottle_ready : 1;
    uint8_t proportional_valve_airpre_ref;
    // uint8_t shooter_is_airpre_ready : 1;
};
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
/* Private function definitions ----------------------------------------------*/

void GimbalShooterComm::encodeG2S(uint8_t tx_data[8])
{
    Gimbal2ShooterPkg pkg = {0};
    pkg.main_board_gimbal_work_state = main_board_data_.gimbal_work_state;
    pkg.shooter_command = gimbal_data_.shoot_command;
    memcpy(tx_data, &pkg, sizeof(pkg));
};
void GimbalShooterComm::decodeG2S(uint8_t rx_data[8])
{
    Gimbal2ShooterPkg pkg = {0};
    memcpy(&pkg, rx_data, sizeof(pkg));
    main_board_data_.gimbal_work_state = WorkState(pkg.main_board_gimbal_work_state);
    gimbal_data_.shoot_command = pkg.shooter_command;
};
void GimbalShooterComm::encodeS2G(uint8_t tx_data[8])
{
    Shooter2GimbalPkg pkg = {0};
    pkg.main_board_shooter_work_state = main_board_data_.shooter_work_state;
    pkg.is_shooter_ready = shooter_data_.is_shooter_ready;
    pkg.is_shooter_pill_ready = shooter_data_.is_shooter_pill_ready;
    pkg.is_airpre_ready = shooter_data_.is_airpre_ready;
    pkg.is_air_bottle_ready = shooter_data_.is_air_bottle_ready;
    pkg.proportional_valve_airpre_ref = shooter_data_.proportional_valve_airpre_ref;
    memcpy(tx_data, &pkg, sizeof(pkg));
};
void GimbalShooterComm::decodeS2G(uint8_t rx_data[8])
{
    Shooter2GimbalPkg pkg = {0};
    memcpy(&pkg, rx_data, sizeof(pkg));
    main_board_data_.shooter_work_state = WorkState(pkg.main_board_shooter_work_state);
    shooter_data_.is_shooter_ready = pkg.is_shooter_ready;
    shooter_data_.is_shooter_pill_ready = pkg.is_shooter_pill_ready;
    shooter_data_.is_airpre_ready = pkg.is_airpre_ready;
    shooter_data_.is_air_bottle_ready = pkg.is_air_bottle_ready;
    shooter_data_.proportional_valve_airpre_ref = pkg.proportional_valve_airpre_ref;
};
}