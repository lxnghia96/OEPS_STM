#include "gpio_user.h"
#include "main.h"

void InitIoPinOutput(GPIO_TypeDef * portName, uint16_t pinName)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = pinName;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(portName, &GPIO_InitStruct);
}

void InitIoPinInput(GPIO_TypeDef * portName, uint16_t pinName)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = pinName;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(portName, &GPIO_InitStruct);
}
