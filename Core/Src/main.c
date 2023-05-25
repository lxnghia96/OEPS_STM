/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "spi_software.h"
#include "HEFlash.h"
#include "string.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEFLASH_SIZE 32
#define ADC_INTERNAL false
#define V_REF_3V3 (float)3.11
#define V_REF_2V5 (float)2.47
#define V_OFFSET (float)0.006
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

typedef enum Adc_Interal
{
  V_MEASURE = 0U,
  I_MEASURE,
  WAIT_MEASURE
};

typedef enum dpv_direct
{
  RAISING  = 0U,
  FALLING,
};

typedef struct Dpv_Info_t
{
  uint8_t segment;
  uint16_t init_potential;
  uint8_t direct;
  uint16_t upper_vertex_potential;
  uint16_t lower_potential;
  uint16_t final_potential;
  uint16_t height;
  uint16_t width;
  uint16_t period;
  uint16_t increments;
  uint16_t pre_pulse;
  uint16_t post_pulse;
}Dpv_Info_t;


typedef Dpv_Info_t dpv_info;

dpv_info dpv_infor;






uint8_t adc_Internal[6];
enum Adc_Interal adcConvert = WAIT_MEASURE;
bool isActiveAdcInternal = false;
volatile bool isUpdate = true;
static const uint8_t *received_data;
static uint8_t received_data_length;
static uint8_t transmit_data[128];
static uint8_t transmit_data_length;
static uint8_t heflashbuffer[HEFLASH_SIZE];
extern uint8_t isRecvData;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint8_t recvDataLen;
volatile uint32_t adc_value = 0U;
uint8_t updateAdcInternal(void);
void command_set_dac_cal_Internal(const uint8_t *dac_cal_data);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Adc_read_V_Measure(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void Adc_read_I_Measure(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}


void InitializeIO()
{
  HAL_TIM_Base_Start(&htim1);
  HAL_GPIO_WritePin(MODE_SW_GPIO_Port, MODE_SW_Pin, GPIO_PIN_RESET); // initialize mode to potentiostatic
  HAL_GPIO_WritePin(CELL_ON_GPIO_Port, CELL_ON_Pin, GPIO_PIN_SET);   // initialize cell to off position
  HAL_GPIO_WritePin(RANGE1_GPIO_Port, RANGE1_Pin, GPIO_PIN_RESET);   // initialize range to range 1
  HAL_GPIO_WritePin(RANGE2_GPIO_Port, RANGE2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RANGE3_GPIO_Port, RANGE3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RANGE4_GPIO_Port, RANGE4_Pin, GPIO_PIN_SET);
  InitializeSPI();
  HAL_Delay(25); // power-up delay - necessary for DAC1220
  DAC1220_Reset();
  HAL_Delay(25);
  DAC1220_Init();
  HEFLASH_readBlock(heflashbuffer, 2, HEFLASH_SIZE);                            // get dac calibration
  DAC1220_Write3Bytes(8, heflashbuffer[0], heflashbuffer[1], heflashbuffer[2]); // apply dac calibration
  DAC1220_Write3Bytes(12, heflashbuffer[3], heflashbuffer[4], heflashbuffer[5]);

}

void command_unknown()
{
  const uint8_t *reply = "?";
  strcpy(transmit_data, reply);
  transmit_data_length = strlen(reply);
}

void send_OK()
{
  const uint8_t *reply = "OK";
  strcpy(transmit_data, reply);
  transmit_data_length = strlen(reply);
}

void command_cell_on()
{
  HAL_GPIO_WritePin(CELL_ON_GPIO_Port, CELL_ON_Pin, GPIO_PIN_RESET);
  send_OK();
}

void command_cell_off()
{
  HAL_GPIO_WritePin(CELL_ON_GPIO_Port, CELL_ON_Pin, GPIO_PIN_SET);
  send_OK();
}

void command_mode_potentiostatic()
{
  HAL_GPIO_WritePin(MODE_SW_GPIO_Port, MODE_SW_Pin, GPIO_PIN_RESET);
  send_OK();
}

void command_mode_galvanostatic()
{
  HAL_GPIO_WritePin(MODE_SW_GPIO_Port, MODE_SW_Pin, GPIO_PIN_SET);
  send_OK();
}

void command_range1()
{
  HAL_GPIO_WritePin(RANGE1_GPIO_Port, RANGE1_Pin, GPIO_PIN_RESET);
  HAL_Delay(10); // make the new relay setting before breaking the old one
  HAL_GPIO_WritePin(RANGE2_GPIO_Port, RANGE2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RANGE3_GPIO_Port, RANGE3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RANGE4_GPIO_Port, RANGE4_Pin, GPIO_PIN_SET);
  send_OK();
}

void command_range2()
{
  HAL_GPIO_WritePin(RANGE2_GPIO_Port, RANGE2_Pin, GPIO_PIN_RESET);
  HAL_Delay(10); // make the new relay setting before breaking the old one
  HAL_GPIO_WritePin(RANGE1_GPIO_Port, RANGE1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RANGE3_GPIO_Port, RANGE3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RANGE4_GPIO_Port, RANGE4_Pin, GPIO_PIN_SET);
  send_OK();
}

void command_range3()
{
  HAL_GPIO_WritePin(RANGE3_GPIO_Port, RANGE3_Pin, GPIO_PIN_RESET);
  HAL_Delay(10); // make the new relay setting before breaking the old one
  HAL_GPIO_WritePin(RANGE1_GPIO_Port, RANGE1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RANGE2_GPIO_Port, RANGE2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RANGE4_GPIO_Port, RANGE4_Pin, GPIO_PIN_SET);
  send_OK();
}

void command_range4()
{
  HAL_GPIO_WritePin(RANGE4_GPIO_Port, RANGE4_Pin, GPIO_PIN_RESET);
  HAL_Delay(10); // make the new relay setting before breaking the old one
  HAL_GPIO_WritePin(RANGE1_GPIO_Port, RANGE1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RANGE2_GPIO_Port, RANGE2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RANGE3_GPIO_Port, RANGE3_Pin, GPIO_PIN_SET);
  send_OK();
}

void command_set_dac_Internal(const uint8_t *dac_data)
{
  uint32_t code = 0x00;
  uint32_t dacOut;
  code |= dac_data[0] << 12;
  code |= dac_data[1] << 4;
  code |= dac_data[2] >> 4;
  dacOut = ((float)code / pow(2, 19) * V_REF_2V5 - V_OFFSET) / V_REF_3V3 * 4095;
  /* Convert to vref 3.3V*/

  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)(dacOut));
  send_OK();
}

void command_set_dac(const uint8_t *dac_data)
{
  DAC1220_Write3Bytes(0, dac_data[0], dac_data[1], dac_data[2]);
  send_OK();
}

void command_calibrate_dac()
{
  DAC1220_SelfCal();
  HAL_Delay(500); // wait until calibration is finished
  uint8_t data[6];
  DAC1220_Read3Bytes(8, data, data + 1, data + 2); // get calibration data
  DAC1220_Read3Bytes(12, data + 3, data + 4, data + 5);
  HEFLASH_writeBlock(2, data, 6); // save calibration data to HEFLASH
  send_OK();
}

void command_calibrate_dac_Internal()
{
  DAC1220_SelfCal();
  HAL_Delay(500); // wait until calibration is finished
  uint8_t data[6];
  DAC1220_Read3Bytes(8, data, data + 1, data + 2); // get calibration data
  DAC1220_Read3Bytes(12, data + 3, data + 4, data + 5);
  HEFLASH_writeBlock(2, data, 6); // save calibration data to HEFLASH
  send_OK();
}

void command_read_adc_Internal()
{
	if(updateAdcInternal())
	{
		transmit_data_length=6;
		memcpy(transmit_data, adc_Internal, transmit_data_length);
	}
	else
	{
		const uint8_t* reply = "WAIT";
		strcpy(transmit_data, reply);
		transmit_data_length = strlen(reply);
	}
}

void command_read_adc()
{
  uint8_t adc_data[6];
  if (MCP3550_Read(adc_data))
  {
    transmit_data_length = 6;
    memcpy(transmit_data, adc_data, transmit_data_length);
  }
  else
  {
    const uint8_t *reply = "WAIT";
    strcpy(transmit_data, reply);
    transmit_data_length = strlen(reply);
  }
}

void command_read_offset()
{
  HEFLASH_readBlock(heflashbuffer, 1, HEFLASH_SIZE);
  transmit_data_length = 6;
  memcpy(transmit_data, heflashbuffer, transmit_data_length);
}

void command_save_offset(const uint8_t *offset_data)
{
  HEFLASH_writeBlock(1, offset_data, 6);
}

void command_read_shuntcalibration()
{
  HEFLASH_readBlock(heflashbuffer, 3, HEFLASH_SIZE);
  transmit_data_length = 8;
  memcpy(transmit_data, heflashbuffer, transmit_data_length);
}

void command_save_shuntcalibration(const uint8_t *shuntcalibration_data)
{
  HEFLASH_writeBlock(3, shuntcalibration_data, 8);
  send_OK();
}

void command_read_dac_cal()
{
  HEFLASH_readBlock(heflashbuffer, 2, HEFLASH_SIZE);
  transmit_data_length = 6;
  memcpy(transmit_data, heflashbuffer, transmit_data_length);
}

void command_set_dac_cal(const uint8_t *dac_cal_data)
{
  HEFLASH_writeBlock(2, dac_cal_data, 6);
  DAC1220_Write3Bytes(8, dac_cal_data[0], dac_cal_data[1], dac_cal_data[2]);
  DAC1220_Write3Bytes(12, dac_cal_data[3], dac_cal_data[4], dac_cal_data[5]);
  send_OK();
}

void command_set_dac_cal_Internal(const uint8_t *dac_cal_data)
{
  HEFLASH_writeBlock(2, dac_cal_data, 6);
  DAC1220_Write3Bytes(8, dac_cal_data[0], dac_cal_data[1], dac_cal_data[2]);
  DAC1220_Write3Bytes(12, dac_cal_data[3], dac_cal_data[4], dac_cal_data[5]);
  send_OK();
}

void measure_dpv()
{

}

void interpret_command()
{
  if (received_data_length == 7 && strncmp(received_data, "CELL ON", 7) == 0)
    command_cell_on();
  else if (received_data_length == 8 && strncmp(received_data, "CELL OFF", 8) == 0)
    command_cell_off();
  else if (received_data_length == 14 && strncmp(received_data, "POTENTIOSTATIC", 14) == 0)
    command_mode_potentiostatic();
  else if (received_data_length == 13 && strncmp(received_data, "GALVANOSTATIC", 13) == 0)
    command_mode_galvanostatic();
  else if (received_data_length == 7 && strncmp(received_data, "RANGE 1", 7) == 0)
    command_range1();
  else if (received_data_length == 7 && strncmp(received_data, "RANGE 2", 7) == 0)
    command_range2();
  else if (received_data_length == 7 && strncmp(received_data, "RANGE 3", 7) == 0)
    command_range3();
  else if (received_data_length == 7 && strncmp(received_data, "RANGE 4", 7) == 0)
    command_range4();
  else if (received_data_length == 10 && strncmp(received_data, "DACSET ", 7) == 0)
#if ADC_INTERNAL
    command_set_dac_Internal(received_data + 7);
#else
    command_set_dac(received_data + 7);
#endif
  else if (received_data_length == 6 && strncmp(received_data, "DACCAL", 6) == 0)
#if ADC_INTERNAL
    command_calibrate_dac_Internal();
#else
    command_calibrate_dac();
#endif
  else if (received_data_length == 7 && strncmp(received_data, "ADCREAD", 7) == 0)
#if ADC_INTERNAL
    command_read_adc_Internal();
#else
    command_read_adc();
#endif
  else if (received_data_length == 10 && strncmp(received_data, "OFFSETREAD", 10) == 0)
    command_read_offset();
  else if (received_data_length == 17 && strncmp(received_data, "OFFSETSAVE ", 11) == 0)
    command_save_offset(received_data + 11);
  else if (received_data_length == 9 && strncmp(received_data, "DACCALGET", 9) == 0)
    command_read_dac_cal();
  else if (received_data_length == 16 && strncmp(received_data, "DACCALSET ", 10) == 0)
#if ADC_INTERNAL
    command_set_dac_cal_Internal(received_data + 10);
#else
    command_set_dac_cal(received_data + 10);
#endif
  else if (received_data_length == 12 && strncmp(received_data, "SHUNTCALREAD", 12) == 0)
    command_read_shuntcalibration();
  else if (received_data_length == 21 && strncmp(received_data, "SHUNTCALSAVE ", 13) == 0)
    command_save_shuntcalibration(received_data + 13);
  else if(received_data_length == 21 && strncmp(received_data, "DPV_MEASURE", 11) == 0)
    measure_dpv();
  else
    command_unknown();
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	float temp;
  adc_value = HAL_ADC_GetValue(hadc);
  temp = ((float)adc_value / (float)4095 * V_REF_3V3 - V_REF_2V5);
  adc_value = (uint32_t) (temp/V_REF_2V5*2097153);
  if (adcConvert == V_MEASURE)
  {
    /* update three byte 0 1 2*/
    adc_Internal[0] = (uint8_t)((adc_value >> 16) & 0xFF);
    adc_Internal[1] = (uint8_t)((adc_value >> 8) & 0xFF);
    adc_Internal[2] = (uint8_t)((adc_value >> 0) & 0xFF);
  }
  else if (adcConvert == I_MEASURE)
  {
    /* update three byte 3 4 5*/
    adc_Internal[3] = (uint8_t)((adc_value >> 16) & 0xFF);
    adc_Internal[4] = (uint8_t)((adc_value >> 8) & 0xFF);
    adc_Internal[5] = (uint8_t)((adc_value >> 0) & 0xFF);
  }
  isUpdate = true;
}

uint8_t updateAdcInternal(void)
{
  static uint8_t adcProcess = 0;
  uint8_t retVal = 0;

  if (isActiveAdcInternal == true)
  {
    if (adcProcess == 0)
    {
      adcConvert = V_MEASURE;
      adcProcess = 1;
      isUpdate = false;
      Adc_read_V_Measure();
      HAL_ADC_Start_IT(&hadc);
    }
    else if (adcProcess == 1)
    {
      if (isUpdate == true)
      {
        HAL_ADC_Stop_IT(&hadc);
        adcConvert = I_MEASURE;
        adcProcess = 2;
        isUpdate = false;
        Adc_read_I_Measure();
        HAL_ADC_Start_IT(&hadc);
      }
    }
    else if (adcProcess == 2)
    {
      if (isUpdate == true)
      {
    	 retVal = 1;
        adcProcess = 0;
        HAL_ADC_Stop_IT(&hadc);
      }
    }
    else
    {
    }
  }
  return retVal;
}

void isActiveAdcDacInteral(void)
{
  uint32_t dacOut = 0;
  MX_DAC_Init();
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_GPIO_WritePin(I_E_SWITCH_GPIO_Port, I_E_SWITCH_Pin, GPIO_PIN_SET);
  isActiveAdcInternal = true;

  dacOut = (uint32_t)((float)(V_REF_2V5 - V_OFFSET) / V_REF_3V3 * 4095);
  /* Convert to vref 3.3V*/
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)(dacOut));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* do nothing*/
	HAL_GPIO_TogglePin(TEST_PIN_GPIO_Port, TEST_PIN_Pin);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  /*Stub for test*/
	 dpv_infor.segment = 2;
	 dpv_infor.init_potential = 1000;
	 dpv_infor.direct = RAISING;
	 dpv_infor.upper_vertex_potential = 3000;
	 dpv_infor.lower_potential = 500;
	 dpv_infor.final_potential = 500;
	 dpv_infor.height = 500;
	 dpv_infor.width = 20;
	 dpv_infor.period = 100;
	 dpv_infor.increments = 200;
	 dpv_infor.pre_pulse = 50;
	 dpv_infor.post_pulse = 10;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  InitializeIO();
  HAL_TIM_Base_Start_IT(&htim2);

  //isActiveAdcDacInteral();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Receive data */
    if (1U == isRecvData)
    {
      /* clear flag */
      isRecvData = 0U;

      /* get memory length of received data */
      received_data_length = recvDataLen;

      /* get memory location of received data*/
      received_data = UserRxBufferFS;

      interpret_command();

      CDC_Transmit_FS(transmit_data, transmit_data_length);

    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  // sConfig.Channel = ADC_CHANNEL_6;
  // sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  // sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  // if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  // /** Configure for the selected ADC regular channel to be converted.
  // */
  // sConfig.Channel = ADC_CHANNEL_7;
  // if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 479;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS1_Pin|CS2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SDIO_DAC_Pin | SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MODE_SW_Pin|I_E_SWITCH_Pin|RANGE1_Pin|RANGE2_Pin
                          |RANGE3_Pin|RANGE4_Pin|CELL_ON_Pin|TEST_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS1_Pin CS2_Pin SCK_Pin */
  GPIO_InitStruct.Pin = CS1_Pin|CS2_Pin|SDIO_DAC_Pin|SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SDIO1_Pin SDIO2_Pin */
  GPIO_InitStruct.Pin = SDIO1_Pin|SDIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MODE_SW_Pin RANGE1_Pin RANGE2_Pin RANGE3_Pin
                           RANGE4_Pin CELL_ON_Pin */
  GPIO_InitStruct.Pin = MODE_SW_Pin|RANGE1_Pin|RANGE2_Pin|RANGE3_Pin
                          |RANGE4_Pin|CELL_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : I_E_SWITCH_Pin */
  GPIO_InitStruct.Pin = I_E_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I_E_SWITCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TEST_PIN_Pin */
  GPIO_InitStruct.Pin = TEST_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(TEST_PIN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
