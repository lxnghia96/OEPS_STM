#ifndef __SWSPI16_H
#define __SWSPI16_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "gpio_user.h"
#define LOW 0
#define HIGH 1

#define OUTPUT 0
#define INPUT 1


void InitializeSPI();
uint8_t MCP3550_Read(uint8_t* adc_data);
void DAC1220_Reset();
void DAC1220_Write2Bytes(const uint8_t address, const uint8_t byte1, const uint8_t byte2);
void DAC1220_Write3Bytes(const uint8_t address, const uint8_t byte1, const uint8_t byte2, const uint8_t byte3);
void DAC1220_Read2Bytes(const uint8_t address, uint8_t* byte1, uint8_t* byte2);
void DAC1220_Read3Bytes(const uint8_t address, uint8_t* byte1, uint8_t* byte2, uint8_t* byte3);
void DAC1220_Init();
void DAC1220_SelfCal();
void Read2BytesSPI(uint8_t* data1_byte, uint8_t* data2_byte);
uint8_t ReadByteSPI();
void WriteByteSPI(uint8_t data_byte);
void ClockPulse();
void SPIDelay();
void delay_ns(uint16_t delay);
#endif
