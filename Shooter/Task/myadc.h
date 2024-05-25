#ifndef __MYADC_H__
#define __MYADC_H__

/* External variables --------------------------------------------------------*/
extern float voltage;

/* Exported function prototypes ----------------------------------------------*/
void init_vrefint_reciprocal(void);
float get_battery_voltage(void);

#endif