#ifndef __PROPORTIONAL_VALVE_H
#define __PROPORTIONAL_VALVE_H

extern float voltage;

float GetVoltage(void);
void InitVrefintReciprocal(void);
void DP8201SSetVoltage(float target_voltage);
void DAC5571SetVoltage(float target_voltage);

#endif