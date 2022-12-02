/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint8_t channelA;
	uint8_t channelB;
	uint8_t channelC;
	uint8_t trigger_option;
	uint16_t trigger_value;
	uint8_t trigger_channel;
	uint8_t time_scale;
	uint8_t changed;
} options_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

#define ADC_HALF_BUF_LEN 10000u
#define DATA_TO_SEND_LEN 500u
#define HALF_DATA_TO_SEND_LEN 250u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart3;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
/* USER CODE BEGIN PV */
uint8_t ADC_Buf_A_1[ADC_HALF_BUF_LEN * 2];
uint8_t ADC_Buf_B_1[ADC_HALF_BUF_LEN * 2];
uint8_t ADC_Buf_C_1[ADC_HALF_BUF_LEN * 2];

uint8_t ADC_Buf_A_2[ADC_HALF_BUF_LEN * 2];
uint8_t ADC_Buf_B_2[ADC_HALF_BUF_LEN * 2];
uint8_t ADC_Buf_C_2[ADC_HALF_BUF_LEN * 2];

uint8_t ADC_Buf_A_1_[ADC_HALF_BUF_LEN * 2];
uint8_t ADC_Buf_B_1_[ADC_HALF_BUF_LEN * 2];
uint8_t ADC_Buf_C_1_[ADC_HALF_BUF_LEN * 2];

uint8_t ADC_Buf_A_2_[ADC_HALF_BUF_LEN * 2];
uint8_t ADC_Buf_B_2_[ADC_HALF_BUF_LEN * 2];
uint8_t ADC_Buf_C_2_[ADC_HALF_BUF_LEN * 2];

uint8_t ADC_Tmp_Buf_A[ADC_HALF_BUF_LEN * 3];
uint8_t ADC_Tmp_Buf_B[ADC_HALF_BUF_LEN * 3];
uint8_t ADC_Tmp_Buf_C[ADC_HALF_BUF_LEN * 3];

volatile uint8_t trigger_flag = 0;

volatile uint16_t trigger_A_pos = 0;
volatile uint16_t trigger_B_pos = 0;
volatile uint16_t trigger_C_pos = 0;

volatile uint8_t trigger_quadrant = 0;

volatile uint8_t start_flag = RESET;
volatile uint8_t ready_to_send_flag = 0;

volatile uint16_t buf_A_pos = 0;
volatile uint16_t buf_B_pos = 0;
volatile uint16_t buf_C_pos = 0;

volatile uint8_t buf_A_quadrant = 0;
volatile uint8_t buf_B_quadrant = 0;
volatile uint8_t buf_C_quadrant = 0;
volatile uint8_t first_triggering = 0;

uint8_t data_to_send[DATA_TO_SEND_LEN * 3];

uint8_t dummy_data_to_send[DATA_TO_SEND_LEN * 3];

options_t *options;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM16_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_DAC1_Init(void);
/* USER CODE BEGIN PFP */
void copy_to_tmp_array(void);
void prepare_data_to_send(uint16_t trigger_pos);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */
	/* USER CODE BEGIN Boot_Mode_Sequence_0 */
	int32_t timeout;
	/* USER CODE END Boot_Mode_Sequence_0 */

	/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/* Wait until CPU2 boots and enters in stop mode or timeout*/
	timeout = 0xFFFF;
	while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
	if (timeout < 0)
	{
		Error_Handler();
	}
	/* USER CODE END Boot_Mode_Sequence_1 */
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();
	/* USER CODE BEGIN Boot_Mode_Sequence_2 */
	/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
	 HSEM notification */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/*Take HSEM */
	HAL_HSEM_FastTake(HSEM_ID_0);
	/*Release HSEM in order to notify the CPU2(CM4)*/
	HAL_HSEM_Release(HSEM_ID_0, 0);
	/* wait until CPU2 wakes up from stop mode */
	timeout = 0xFFFF;
	while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
	if (timeout < 0)
	{
		Error_Handler();
	}
	/* USER CODE END Boot_Mode_Sequence_2 */

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART3_UART_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_USB_DEVICE_Init();
	MX_ADC3_Init();
	MX_ADC2_Init();
	MX_TIM16_Init();
	MX_COMP1_Init();
	MX_COMP2_Init();
	MX_DAC1_Init();
	/* USER CODE BEGIN 2 */
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	HAL_TIM_Base_Start_IT(&htim16);

	uint16_t i = 0;
	for (i = 0; i < DATA_TO_SEND_LEN * 3; i++)
	{
		dummy_data_to_send[i] = i & 0xFF;
	}
	options->changed = 0;

	while (!start_flag)
	{
		//waiting for usb connection
	}

	HAL_ADC_Stop_DMA(&hadc1);
	HAL_ADC_Stop_DMA(&hadc2);
	HAL_ADC_Stop_DMA(&hadc3);

	HAL_COMP_Stop(&hcomp1);
	HAL_COMP_Stop(&hcomp2);

	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

	HAL_Delay(500);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_Buf_A_1, ADC_HALF_BUF_LEN * 2);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) ADC_Buf_B_1, ADC_HALF_BUF_LEN * 2);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*) ADC_Buf_C_1, ADC_HALF_BUF_LEN * 2);

//			HAL_COMP_Start_IT(&hcomp1);
//			HAL_COMP_Start_IT(&hcomp2);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	trigger_A_pos = 0;
	trigger_B_pos = 0;

	buf_A_pos = 0;
	buf_B_pos = 0;
	buf_C_pos = 0;

	start_flag = RESET;
	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		if (trigger_flag)
		{
			if (options->trigger_channel == 1)
			{
				if (buf_A_quadrant == (trigger_quadrant + 1) & 3)
				{
					copy_to_tmp_array();
					prepare_data_to_send(trigger_A_pos);
					ready_to_send_flag = 1;
					trigger_flag = 0;
				}
			}
			if (options->trigger_channel == 2)
			{
				if (buf_B_quadrant == (trigger_quadrant + 1) & 3)
				{
					copy_to_tmp_array();
					prepare_data_to_send(trigger_B_pos);
					ready_to_send_flag = 1;
					trigger_flag = 0;
				}
			}
		}

		if (options->changed)
		{

			first_triggering = 0;
			//TODO(Attila): Set everything

			//MUX 1
			HAL_GPIO_WritePin(MUX_1_A_GPIO_Port, MUX_1_A_Pin, !(options->channelA & 1));
			HAL_GPIO_WritePin(MUX_1_B_GPIO_Port, MUX_1_B_Pin, !(options->channelA & 2));
			HAL_GPIO_WritePin(MUX_1_C_GPIO_Port, MUX_1_C_Pin, !(options->channelA & 4));

			//MUX 2
			HAL_GPIO_WritePin(MUX_2_A_GPIO_Port, MUX_2_A_Pin, !(options->channelB & 1));
			HAL_GPIO_WritePin(MUX_2_B_GPIO_Port, MUX_2_B_Pin, !(options->channelB & 2));
			HAL_GPIO_WritePin(MUX_2_C_GPIO_Port, MUX_2_C_Pin, !(options->channelB & 4));

			//DAC setting
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, options->trigger_value);

			//Trigger Mode
			switch (options->trigger_option)
			{
			case 0:
				hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
				hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;

				HAL_COMP_Stop_IT(&hcomp1);
				HAL_COMP_Stop_IT(&hcomp2);

				HAL_COMP_Init(&hcomp1);
				HAL_COMP_Init(&hcomp2);

				trigger_flag = 0;
				HAL_COMP_Start(&hcomp1);
				HAL_COMP_Start(&hcomp2);
				break;

			case 1:
				hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
				hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;

				HAL_COMP_Init(&hcomp1);
				HAL_COMP_Init(&hcomp2);

				trigger_flag = 0;
				HAL_COMP_Start(&hcomp1);
				HAL_COMP_Start(&hcomp2);
				break;

			default:
				HAL_COMP_Stop(&hcomp1);
				HAL_COMP_Stop(&hcomp2);

				break;
			}

			options->changed = 0;
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
	{
	}

	/** Macro to configure the PLL clock source
	 */
	__HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 120;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_1);
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct =
	{ 0 };

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInitStruct.PLL2.PLL2M = 1;
	PeriphClkInitStruct.PLL2.PLL2N = 50;
	PeriphClkInitStruct.PLL2.PLL2P = 8;
	PeriphClkInitStruct.PLL2.PLL2Q = 1;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
	PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
	PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode =
	{ 0 };
	ADC_ChannelConfTypeDef sConfig =
	{ 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_8B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig =
	{ 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc2.Init.Resolution = ADC_RESOLUTION_8B;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc2.Init.LowPowerAutoWait = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
	hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	hadc2.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void)
{

	/* USER CODE BEGIN ADC3_Init 0 */

	/* USER CODE END ADC3_Init 0 */

	ADC_ChannelConfTypeDef sConfig =
	{ 0 };

	/* USER CODE BEGIN ADC3_Init 1 */

	/* USER CODE END ADC3_Init 1 */

	/** Common config
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc3.Init.Resolution = ADC_RESOLUTION_8B;
	hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc3.Init.LowPowerAutoWait = DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
	hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	hadc3.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc3) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */

}

/**
 * @brief COMP1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_COMP1_Init(void)
{

	/* USER CODE BEGIN COMP1_Init 0 */

	/* USER CODE END COMP1_Init 0 */

	/* USER CODE BEGIN COMP1_Init 1 */

	/* USER CODE END COMP1_Init 1 */
	hcomp1.Instance = COMP1;
	hcomp1.Init.InvertingInput = COMP_INPUT_MINUS_DAC1_CH1;
	hcomp1.Init.NonInvertingInput = COMP_INPUT_PLUS_IO2;
	hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
	hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
	hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
	hcomp1.Init.Mode = COMP_POWERMODE_HIGHSPEED;
	hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
	hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
	if (HAL_COMP_Init(&hcomp1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN COMP1_Init 2 */

	/* USER CODE END COMP1_Init 2 */

}

/**
 * @brief COMP2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_COMP2_Init(void)
{

	/* USER CODE BEGIN COMP2_Init 0 */

	/* USER CODE END COMP2_Init 0 */

	/* USER CODE BEGIN COMP2_Init 1 */

	/* USER CODE END COMP2_Init 1 */
	hcomp2.Instance = COMP2;
	hcomp2.Init.InvertingInput = COMP_INPUT_MINUS_DAC1_CH1;
	hcomp2.Init.NonInvertingInput = COMP_INPUT_PLUS_IO1;
	hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
	hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
	hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
	hcomp2.Init.Mode = COMP_POWERMODE_HIGHSPEED;
	hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
	hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
	if (HAL_COMP_Init(&hcomp2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN COMP2_Init 2 */

	/* USER CODE END COMP2_Init 2 */

}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void)
{

	/* USER CODE BEGIN DAC1_Init 0 */

	/* USER CODE END DAC1_Init 0 */

	DAC_ChannelConfTypeDef sConfig =
	{ 0 };

	/* USER CODE BEGIN DAC1_Init 1 */

	/* USER CODE END DAC1_Init 1 */

	/** DAC Initialization
	 */
	hdac1.Instance = DAC1;
	if (HAL_DAC_Init(&hdac1) != HAL_OK)
	{
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN DAC1_Init 2 */

	/* USER CODE END DAC1_Init 2 */

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

	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig =
	{ 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 48 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void)
{

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 2000 - 1;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 2000 - 1;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM16_Init 2 */

	/* USER CODE END TIM16_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 * Configure DMA for memory to memory transfers
 *   hdma_memtomem_dma2_stream0
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
	hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
	hdma_memtomem_dma2_stream0.Init.Request = DMA_REQUEST_MEM2MEM;
	hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
	hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
	hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
	hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
	hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_LOW;
	hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
	if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
	{
		Error_Handler();
	}

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	/* DMA1_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin | MUX_2_A_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, MUX_2_C_Pin | MUX_2_B_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, MUX_1_A_Pin | MUX_1_B_Pin | MUX_1_C_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin MUX_2_A_Pin */
	GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin | MUX_2_A_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
	GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MUX_2_C_Pin MUX_2_B_Pin */
	GPIO_InitStruct.Pin = MUX_2_C_Pin | MUX_2_B_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : PC9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : MUX_1_A_Pin MUX_1_B_Pin MUX_1_C_Pin */
	GPIO_InitStruct.Pin = MUX_1_A_Pin | MUX_1_B_Pin | MUX_1_C_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// Callback: timer has rolled over (60Hz)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Check which version of the timer triggered this callback and toggle LED
	if (htim == &htim16)
	{

		if (ready_to_send_flag)
		{
			//CDC_Transmit_FS((uint8_t*) dummy_data_to_send, DATA_TO_SEND_LEN * 3);
			CDC_Transmit_FS((uint8_t*) data_to_send, DATA_TO_SEND_LEN * 3);
			trigger_flag = 0;
			ready_to_send_flag = 0;
			HAL_COMP_Start_IT(&hcomp1);
			HAL_COMP_Start_IT(&hcomp2);

		}
		//trigger_flag = RESET;

	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{

	if (hadc == &hadc1)
	{
		buf_A_quadrant++;
	}
	if (hadc == &hadc2)
	{
		buf_B_quadrant++;
	}
	if (hadc == &hadc3)
	{
		buf_C_quadrant++;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

	if (hadc == &hadc1)
	{
		buf_A_quadrant++;
		if (buf_A_quadrant >= 4)
		{
			//HAL_ADC_Stop_DMA(&hadc1);
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_Buf_A_1, ADC_HALF_BUF_LEN * 2);
			buf_A_quadrant = 0;
		}
		if (buf_A_quadrant == 2)
		{
			//HAL_ADC_Stop_DMA(&hadc1);
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_Buf_A_2, ADC_HALF_BUF_LEN * 2);
		}
	}
	if (hadc == &hadc2)
	{
		buf_B_quadrant++;
		if (buf_B_quadrant >= 4)
		{
			//HAL_ADC_Stop_DMA(&hadc2);
			HAL_ADC_Start_DMA(&hadc2, (uint32_t*) ADC_Buf_B_1, ADC_HALF_BUF_LEN * 2);
			buf_B_quadrant = 0;
		}
		if (buf_B_quadrant == 2)
		{
			//HAL_ADC_Stop_DMA(&hadc2);
			HAL_ADC_Start_DMA(&hadc2, (uint32_t*) ADC_Buf_B_2, ADC_HALF_BUF_LEN * 2);
		}
	}
	if (hadc == &hadc3)
	{
		buf_C_quadrant++;
		if (buf_C_quadrant >= 4)
		{
			//HAL_ADC_Stop_DMA(&hadc3);
			HAL_ADC_Start_DMA(&hadc3, (uint32_t*) ADC_Buf_C_1, ADC_HALF_BUF_LEN * 2);
			buf_C_quadrant = 0;
		}
		if (buf_C_quadrant == 2)
		{
			//HAL_ADC_Stop_DMA(&hadc3);
			HAL_ADC_Start_DMA(&hadc3, (uint32_t*) ADC_Buf_C_2, ADC_HALF_BUF_LEN * 2);
		}
	}

}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
	if (!first_triggering)
	{
		first_triggering = 1;
		return;
	}
	//TODO(Attila): event handling
	if (hcomp == &hcomp1 && trigger_flag == 0)
	{
		trigger_A_pos = __HAL_DMA_GET_COUNTER(&hdma_adc1);
		if (options->trigger_channel == 1)
		{
			trigger_flag = 1;
			trigger_quadrant = buf_A_quadrant;
			HAL_COMP_Stop(&hcomp1);
		}

	}
	if (hcomp == &hcomp2 && trigger_flag == 0)
	{
		trigger_B_pos = __HAL_DMA_GET_COUNTER(&hdma_adc2);
		if (options->trigger_channel == 2)
		{
			trigger_flag = 1;
			trigger_quadrant = buf_B_quadrant;
			HAL_COMP_Stop(&hcomp2);
		}

	}

}

void copy_to_tmp_array()
{
//	uint32_t k = 0;
//	uint32_t adc_buf_len = 2 * ADC_HALF_BUF_LEN;
//	for(k = 0; k < 2 * ADC_HALF_BUF_LEN; k++){
//		ADC_Buf_A_1[k] = k;
//		ADC_Buf_A_2[k] = k + adc_buf_len;
//
//		ADC_Buf_B_1[k] = k;
//		ADC_Buf_B_2[k] = k + adc_buf_len;
//
//		ADC_Buf_C_1[k] = k;
//		ADC_Buf_C_2[k] = k + adc_buf_len;
//	}
	if (trigger_quadrant == 0)
	{
		memcpy(ADC_Tmp_Buf_A, ADC_Buf_A_2 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_A + ADC_HALF_BUF_LEN, ADC_Buf_A_1, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_A + 2 * ADC_HALF_BUF_LEN, ADC_Buf_A_1 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);

		memcpy(ADC_Tmp_Buf_B, ADC_Buf_B_2 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_B + ADC_HALF_BUF_LEN, ADC_Buf_B_1, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_B + 2 * ADC_HALF_BUF_LEN, ADC_Buf_B_1 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);

		memcpy(ADC_Tmp_Buf_C, ADC_Buf_C_2 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_C + ADC_HALF_BUF_LEN, ADC_Buf_C_1, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_C + 2 * ADC_HALF_BUF_LEN, ADC_Buf_C_1 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);

	}

	if (trigger_quadrant == 1)
	{
		memcpy(ADC_Tmp_Buf_A, ADC_Buf_A_1, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_A + ADC_HALF_BUF_LEN, ADC_Buf_A_1 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_A + 2 * ADC_HALF_BUF_LEN, ADC_Buf_A_2, ADC_HALF_BUF_LEN);

		memcpy(ADC_Tmp_Buf_B, ADC_Buf_B_1, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_B + ADC_HALF_BUF_LEN, ADC_Buf_B_1 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_B + 2 * ADC_HALF_BUF_LEN, ADC_Buf_B_2, ADC_HALF_BUF_LEN);

		memcpy(ADC_Tmp_Buf_C, ADC_Buf_C_1, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_C + ADC_HALF_BUF_LEN, ADC_Buf_C_1 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_C + 2 * ADC_HALF_BUF_LEN, ADC_Buf_C_2, ADC_HALF_BUF_LEN);

	}

	if (trigger_quadrant == 2)
	{
		memcpy(ADC_Tmp_Buf_A, ADC_Buf_A_1 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_A + ADC_HALF_BUF_LEN, ADC_Buf_A_2, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_A + 2 * ADC_HALF_BUF_LEN, ADC_Buf_A_2 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);

		memcpy(ADC_Tmp_Buf_B, ADC_Buf_B_1 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_B + ADC_HALF_BUF_LEN, ADC_Buf_B_2, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_B + 2 * ADC_HALF_BUF_LEN, ADC_Buf_B_2 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);

		memcpy(ADC_Tmp_Buf_C, ADC_Buf_C_1 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_C + ADC_HALF_BUF_LEN, ADC_Buf_C_2, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_C + 2 * ADC_HALF_BUF_LEN, ADC_Buf_C_2 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);

	}

	if (trigger_quadrant == 3)
	{
		memcpy(ADC_Tmp_Buf_A, ADC_Buf_A_2, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_A + ADC_HALF_BUF_LEN, ADC_Buf_A_2 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_A + 2 * ADC_HALF_BUF_LEN, ADC_Buf_A_1, ADC_HALF_BUF_LEN);

		memcpy(ADC_Tmp_Buf_B, ADC_Buf_B_2, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_B + ADC_HALF_BUF_LEN, ADC_Buf_B_2 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_B + 2 * ADC_HALF_BUF_LEN, ADC_Buf_B_1, ADC_HALF_BUF_LEN);

		memcpy(ADC_Tmp_Buf_C, ADC_Buf_C_2, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_C + ADC_HALF_BUF_LEN, ADC_Buf_C_2 + ADC_HALF_BUF_LEN, ADC_HALF_BUF_LEN);
		memcpy(ADC_Tmp_Buf_C + 2 * ADC_HALF_BUF_LEN, ADC_Buf_C_1, ADC_HALF_BUF_LEN);
	}
}

void prepare_data_to_send(uint16_t trigger_pos)
{
	if (trigger_quadrant == 1 || trigger_quadrant == 3)
	{
		trigger_pos += ADC_HALF_BUF_LEN;
	}
	uint32_t i = trigger_pos;
	uint8_t step = options->time_scale + 1;
	uint16_t interval = DATA_TO_SEND_LEN * step - 1;
	uint16_t half_interval = interval / 2;
	uint16_t half_data_to_send_len = DATA_TO_SEND_LEN / 2;
	data_to_send[0] = 0;
	data_to_send[DATA_TO_SEND_LEN] = 1;
	data_to_send[DATA_TO_SEND_LEN * 2] = 2;
	data_to_send[250] = ADC_Tmp_Buf_A[trigger_pos];
	data_to_send[DATA_TO_SEND_LEN + 250] = ADC_Tmp_Buf_B[trigger_pos];
	data_to_send[DATA_TO_SEND_LEN * 2 + 250] = ADC_Tmp_Buf_C[trigger_pos];

//	uint32_t k = 0; //TODO fill with dummy data
//	for(k = 0; k < ADC_HALF_BUF_LEN * 3 ; k++){
//		ADC_Tmp_Buf_A[k] = k;
//		ADC_Tmp_Buf_B[k] = k;
//		ADC_Tmp_Buf_C[k] = k;
//	}

	for (i = 1; i < half_data_to_send_len; i++ )
	{
		data_to_send[i] = ADC_Tmp_Buf_A[trigger_pos + i * step];
		data_to_send[DATA_TO_SEND_LEN + i] = ADC_Tmp_Buf_B[trigger_pos + i * step];
		data_to_send[DATA_TO_SEND_LEN + DATA_TO_SEND_LEN + i] = ADC_Tmp_Buf_C[trigger_pos + i * step];

	}

	for (i = half_data_to_send_len; i < DATA_TO_SEND_LEN; i++)
	{
		data_to_send[i] = ADC_Tmp_Buf_A[trigger_pos + i * step];
		data_to_send[i + DATA_TO_SEND_LEN] = ADC_Tmp_Buf_B[trigger_pos + i * step];
		data_to_send[i + DATA_TO_SEND_LEN + DATA_TO_SEND_LEN] = ADC_Tmp_Buf_C[trigger_pos + i * step];

	}

	trigger_flag = 0;
	ready_to_send_flag = 1;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
