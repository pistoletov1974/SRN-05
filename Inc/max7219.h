#ifndef __MAX7219_H__
#define __MAX7219_H__

#include "stm32f4xx_hal.h"

void init_max7219(uint8_t intensivity);
void max7219_setIntensivity(uint8_t intensivity);
void displayNumberLow (uint16_t number);
void displayNumberHigh (uint16_t number);

#endif /* __MAX7219_H__ */
