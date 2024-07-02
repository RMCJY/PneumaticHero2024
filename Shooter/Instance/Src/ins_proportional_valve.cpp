/*
 * @Author: RMCJY 1409947012@qq.com
 * @Date: 2024-07-02 22:33:21
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-07-02 22:51:20
 * @FilePath: \Shooter\Instance\Src\ins_proportional_valve.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_proportional_valve.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hero::ProportionalValve unique_pv = hero::ProportionalValve();
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hero::ProportionalValve* CreateProportionalValve(void) { return &unique_pv; };
/* Private function definitions ----------------------------------------------*/
