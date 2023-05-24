#include "HEFlash.h"
#include "string.h"

uint8_t HEFLASH_writeBlock( uint8_t radd, const uint8_t* data, uint8_t count)
{
    uint8_t writeData[128];
    uint64_t tempValue;
    uint8_t counter;
    uint32_t add = radd * FLASH_PAGE_SIZE + HEFLASH_START;

    memset(writeData, 0xFF, 128);
    memcpy(writeData, data, count);

    HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInit;
	EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInit.PageAddress = add;
	EraseInit.NbPages = 1;
	uint32_t PageError = 0;
	HAL_FLASHEx_Erase(&EraseInit, &PageError);

    for(counter = 0; counter < 16; counter++)
    {
        add = add + counter*64;
        memcpy(&tempValue, &writeData[counter*8], 8);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, add, tempValue);
    }
    HAL_FLASH_Lock();

    return 0;

} 


uint8_t HEFLASH_readBlock( uint8_t *buffer, uint8_t radd, uint8_t count)
{
    uint8_t returnData[128];
    uint32_t tempData;
    uint8_t counter;

    uint32_t add = radd * FLASH_PAGE_SIZE + HEFLASH_START;

    memset(returnData, 0xFF, 128);

    for(counter = 0; counter < 32; counter++)
    {
        add = add + counter * 4;
        tempData = *(__IO uint32_t *)(add);
        memcpy(&returnData[counter * 4], &tempData, 4);
    }

    memcpy(buffer, returnData, count);
    
    // 4. success
    return 0;

}
