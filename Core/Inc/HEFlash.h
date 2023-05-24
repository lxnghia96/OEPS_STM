#include "main.h"

#define HEFLASH_START ((uint32_t)0x0800D000)

uint8_t HEFLASH_writeBlock( uint8_t radd, const uint8_t* data, uint8_t count);
uint8_t HEFLASH_readBlock( uint8_t *buffer, uint8_t radd, uint8_t count);

