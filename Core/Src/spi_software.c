#include "spi_software.h"
#include "main.h"


extern TIM_HandleTypeDef htim1;

void InitializeSPI()
{
	// Initialize the chip select lines as inactive
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
	// Configure the chip select lines as outputs
	InitIoPinOutput(CS1_GPIO_Port, CS1_Pin);
	InitIoPinOutput(CS2_GPIO_Port, CS2_Pin);
	// The clock line should be an output; initialize it to a low state
	InitIoPinOutput(SCK_GPIO_Port, SCK_Pin);
	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
	// Initialize the data lines as inputs
}

uint8_t MCP3550_Read(uint8_t* adc_data)
{
	uint8_t data_ready = 0;
	// Poll conversion status
	HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_RESET);
	SPIDelay();
	if(!HAL_GPIO_ReadPin(SDIO1_GPIO_Port, SDIO1_Pin)) // conversions are ready
	{
		Read2BytesSPI(adc_data,adc_data+3);
		Read2BytesSPI(adc_data+1,adc_data+4);
		Read2BytesSPI(adc_data+2,adc_data+5);
		data_ready = 1;
		// Initiate a new conversion
		HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
		SPIDelay();
		HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_RESET);
		SPIDelay();
	}
	HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
	SPIDelay();
	return data_ready;
}

void DAC1220_Reset()
{
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
	SPIDelay();
	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
	delay_ns(4000);
	delay_ns(4500);
	delay_ns(4500);
	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
	delay_ns(700);
	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
	delay_ns(5000);
	delay_ns(10000);
	delay_ns(10000);
	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
	delay_ns(700);
	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
	delay_ns(20000);
	delay_ns(20000);
	delay_ns(3000);
	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
	SPIDelay();
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
	SPIDelay();
}

void DAC1220_Write2Bytes(const uint8_t address, const uint8_t byte1, const uint8_t byte2)
{
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
	SPIDelay();
	InitIoPinOutput(SDIO_DAC_GPIO_Port, SDIO_DAC_Pin);
	WriteByteSPI(32+address);
	WriteByteSPI(byte1);
	WriteByteSPI(byte2);
	InitIoPinInput(SDIO_DAC_GPIO_Port, SDIO_DAC_Pin);
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
	SPIDelay();
}

void DAC1220_Write3Bytes(const uint8_t address, const uint8_t byte1, const uint8_t byte2, const uint8_t byte3)
{
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
	SPIDelay();
	InitIoPinOutput(SDIO_DAC_GPIO_Port, SDIO_DAC_Pin);
	WriteByteSPI(64+address);
	WriteByteSPI(byte1);
	WriteByteSPI(byte2);
	WriteByteSPI(byte3);
	InitIoPinInput(SDIO_DAC_GPIO_Port, SDIO_DAC_Pin);
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
	SPIDelay();
}

void DAC1220_Read2Bytes(const uint8_t address, uint8_t* byte1, uint8_t* byte2)
{
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
	SPIDelay();
	InitIoPinOutput(SDIO_DAC_GPIO_Port, SDIO_DAC_Pin);
	WriteByteSPI(160+address);
	InitIoPinInput(SDIO_DAC_GPIO_Port, SDIO_DAC_Pin);
	SPIDelay();
	*byte1 = ReadByteSPI();
	*byte2 = ReadByteSPI();
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
	SPIDelay();
}

void DAC1220_Read3Bytes(const uint8_t address, uint8_t* byte1, uint8_t* byte2, uint8_t* byte3)
{
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
	SPIDelay();
	InitIoPinOutput(SDIO_DAC_GPIO_Port, SDIO_DAC_Pin);
	WriteByteSPI(192+address);
	InitIoPinInput(SDIO_DAC_GPIO_Port, SDIO_DAC_Pin);
	SPIDelay();
	*byte1 = ReadByteSPI();
	*byte2 = ReadByteSPI();
	*byte3 = ReadByteSPI();
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
	SPIDelay();
}

void DAC1220_Init()
{

	DAC1220_Write2Bytes(4, 32, 160); // command register: 20-bit resolution; straight binary
	DAC1220_Write3Bytes(0, 128, 0, 0); // set midscale output
}

void DAC1220_SelfCal()
{
	DAC1220_Write2Bytes(4, 32, 161); // command register: 20-bit resolution; straight binary, self calibration mode
}

void Read2BytesSPI(uint8_t* data1_byte, uint8_t* data2_byte)
{
	*data1_byte = 0;      // data to be read in
	*data2_byte = 0;      // data to be read in
	uint8_t bit_counter = 8;     // set bit count for byte
	do
	{
		ClockPulse();			  // generate a clock pulse
		*data1_byte <<= 1;        // shift composed byte by 1
		*data2_byte <<= 1;
		*data1_byte &= 0xFE;      // clear bit 0
		*data2_byte &= 0xFE;
		if(HAL_GPIO_ReadPin(SDIO1_GPIO_Port, SDIO1_Pin))            // is data line high
			*data1_byte |= 0x01;  // set bit 0 to logic 1
		if(HAL_GPIO_ReadPin(SDIO2_GPIO_Port, SDIO2_Pin))            // is data line high
			*data2_byte |= 0x01;  // set bit 0 to logic 1
	} while (--bit_counter);     // repeat until 8 bits have been acquired
}

uint8_t ReadByteSPI()
{
	uint8_t data_byte = 0;      // data to be read in
	uint8_t bit_counter = 8;     // set bit count for byte
	do
	{
		ClockPulse();            // generate a clock pulse
		data_byte <<= 1;         // shift composed byte by 1
		data_byte &= 0xFE;       // clear bit 0
		if(HAL_GPIO_ReadPin(SDIO_DAC_GPIO_Port, SDIO_DAC_Pin))            // is data line high
			data_byte |= 0x01;   // set bit 0 to logic 1
	} while (--bit_counter);     // repeat until 8 bits have been acquired
	return data_byte;
}

void WriteByteSPI(uint8_t data_byte)
{
	uint8_t bit_counter = 8;     // set bit count for byte
	do
	{
		HAL_GPIO_WritePin(SDIO_DAC_GPIO_Port, SDIO_DAC_Pin, ((data_byte&0x80)?HIGH:LOW));  // output most significant bit
		ClockPulse();                           // generate a clock pulse
		data_byte <<= 1;                        // shift byte to the left
	} while (--bit_counter);                    // repeat until 8 bits have been transmitted
}

void ClockPulse()
{
	// Generate clock pulse
	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
	SPIDelay();
	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
	SPIDelay();
}

void SPIDelay()
{
	delay_ns(700); // delay of 100 instruction cycles (=17 us at Fosc=48 MHz)
}

void delay_ns(uint16_t delay)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while( __HAL_TIM_GET_COUNTER(&htim1) < delay);
}



