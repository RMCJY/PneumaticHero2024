/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-01-29 15:59:57
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-01-29 20:18:04
 * @FilePath: \UPER\Task\pid.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _PID_H_
#define _PID_H_	 

/* ------------------------------ Type Definition ------------------------------ */
typedef struct
{
	float ref;//输入：系统待调节量的给定值
	float fdb;//输入：系统待调节量的反馈值
	float inte;//积分值
	float err[2];
	float kp;
	float ki;
	float kd;
	float componentKp;
	float componentKi;
	float componentKd;
	float componentKpMax;
	float componentKiMax;
	float componentKdMax;
	float output;
	float outputMax;
} PID_Regulator_t;

typedef struct
{
	PID_Regulator_t Position;
	PID_Regulator_t Speed;
} PID_Regulator_Double_Loop_t;

/* ------------------------------ Extern Global Variable ------------------------------ */
extern PID_Regulator_t Air_Pressure_PID;

/* ------------------------------ Function Declaration ------------------------------ */
void PID_Init(PID_Regulator_t *pid,float kp,float ki,float kd,float componentKpMax,float componentKiMax,float componentKdMax,float outputMax);
void PID_Calc(PID_Regulator_t *pid);
void PID_ALL_Init(void);

#endif
