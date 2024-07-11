/* ------------------------------ Include ------------------------------ */
#include "pid.h"

/* ------------------------------ Macro Definition ------------------------------ */
#define   FEEDFLOATMAX 15000		//15000


#define		AIR_PRESSURE_KP_DEFAULTS		0
#define		AIR_PRESSURE_KI_DEFAULTS		0
#define		AIR_PRESSURE_KD_DEFAULTS		0
#define		AIR_PRESSURE_KP_MAX				5
#define		AIR_PRESSURE_KI_MAX				5
#define		AIR_PRESSURE_KD_MAX				5
#define		AIR_PRESSURE_OUTPUTMAX			5

/* ------------------------------ Global Variable ------------------------------ */

PID_Regulator_t Air_Pressure_PID;

/* ------------------------------ Function Definition ------------------------------ */
void PID_Calc(PID_Regulator_t *pid)
{
	pid->err[0] = pid->err[1];
	pid->err[1] = pid->ref - pid->fdb;
	pid->inte += pid->err[1];
		
	
	pid->componentKp  = pid->kp * pid->err[1];                  // error[1] 上次误差
	pid->componentKi  = pid->ki * pid->inte;                    // error[2] 本次误差
	pid->componentKd  = pid->kd * (pid->err[1] - pid->err[0]);  // error[0] 上上次误差
	
	if(pid->componentKp > pid->componentKpMax)
		pid->componentKp = pid->componentKpMax;
	else if (pid->componentKp < -pid->componentKpMax)
		pid->componentKp = -pid->componentKpMax;
		
	if(pid->componentKi > pid->componentKiMax)
		pid->componentKi = pid->componentKiMax;
	else if (pid->componentKi < -pid->componentKiMax)
		pid->componentKi = -pid->componentKiMax;
	
	pid->output = pid->componentKp + pid->componentKi+ pid->componentKd;
	
	if(pid->output > pid->outputMax)
		pid->output = pid->outputMax;
	else if (pid->output < -pid->outputMax)
		pid->output = -pid->outputMax;	
}


void PID_Init(PID_Regulator_t *pid,float kp,float ki,float kd,float componentKpMax,float componentKiMax,float componentKdMax,float outputMax)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->inte = 0;
	//#感觉加上会更好
	pid->err[0] = 0;
	pid->err[1] = 0;

	pid->componentKpMax = componentKpMax;
	pid->componentKiMax = componentKiMax;
	pid->componentKdMax = componentKdMax;
	pid->outputMax = outputMax;
}

void PID_ALL_Init(void)
{

	PID_Init(&Air_Pressure_PID,AIR_PRESSURE_KP_DEFAULTS,AIR_PRESSURE_KI_DEFAULTS,AIR_PRESSURE_KD_DEFAULTS,\
					AIR_PRESSURE_KP_MAX,AIR_PRESSURE_KI_MAX,AIR_PRESSURE_KD_MAX,AIR_PRESSURE_OUTPUTMAX);
}
