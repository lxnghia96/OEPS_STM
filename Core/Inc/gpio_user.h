#ifndef __GPIO_USER_H
#define __GPIO_USER_H
#include "stm32f072xb.h"

void InitIoPinOutput(GPIO_TypeDef * portName, uint16_t pinName);
void InitIoPinInput(GPIO_TypeDef * portName, uint16_t pinName);
#endif
