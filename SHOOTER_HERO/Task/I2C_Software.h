#ifndef __I2C_SOFTWARE_H
#define __I2C_SOFTWARE_H

#include "main.h"

uint8_t I2CWriteData(uint8_t dev_addr, uint8_t *data, uint8_t len);

#endif