/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "ssd1306.h"
#include "usbd_cdc_if.h"

#define RES_DEVIDER 0.5	//coef devider for Vbat
#define SAMPLES 4000	//sample count (2 channels, 2 seconds)
#define R_SHUNT 0.2		//shunt resistance
#define V_REF 3300	//reference voltage ADC
#define HIGH_C 10	//high current IN amp coef
#define LOW_C 10000	//low current IN amp coef

#define  LOW_PR (float)(V_REF/4095/LOW_C/R_SHUNT)
#define  HIGH_PR (float)(V_REF/4095/HIGH_C/R_SHUNT)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint32_t Count;
	float Summ;
	float Zero_Level;
	float Current;
}ADC_Channel;

ADC_Channel HIGH_Amp, LOW_Amp;
uint16_t ADC_RAW[SAMPLES] = {0};
/*PA1 - high current IN
Imin = 0,000857 A = 0,857 mA
Imax = 3,5106 A

PA2 - low corrent IN
Imin = 0,0004028 mA = 402,8 nA
Imax = 0,00165 A = 1,65 mA*/

volatile bool FT_flag;
volatile bool HT_flag;
volatile uint8_t DSP_flag = 0;

float ACT_current = 0;
float MAX_current = 0;
float MIN_current = 0;
float mAh_consumed = 0;
float MAX_current_ALLTime = 0;

uint32_t lowmax,highmax;

unsigned int Time_seconds = 0;
uint8_t 	Clock_Days = 0;
uint8_t		Clock_Hours = 0;
uint8_t		Clock_Minutes = 0;
uint8_t		Clock_Seconds = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Device_CFG(void);
void Amp_Calibration(void);
void Display_Data(void);
void Power_Calc(uint16_t start_sample);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  Device_CFG();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(FT_flag){
		  Power_Calc((uint16_t) SAMPLES/2);
		  Display_Data();
		  FT_flag = 0;
	  }

	  if(HT_flag){
		  Power_Calc((uint16_t) 0);
		  Display_Data();
		  HT_flag = 0;
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(72000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_8);
  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL_DIV_1_5);
}

/* USER CODE BEGIN 4 */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
    /* USER CODE BEGIN LL_EXTI_LINE_0 */
    MAX_current = 0;
    MIN_current = 0;
    ACT_current = 0;
    mAh_consumed = 0;
	Time_seconds = 0;
	MAX_current_ALLTime = 0;
	Clock_Days = Clock_Hours = Clock_Minutes = Clock_Seconds = 0;
    /* USER CODE END LL_EXTI_LINE_0 */
  }
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

void Power_Calc(uint16_t start_sample)
{
	HIGH_Amp.Summ = LOW_Amp.Summ = 0;
	HIGH_Amp.Count = LOW_Amp.Count = 0;
	MIN_current = MAX_current = 0;

	uint16_t MIN_low = 4095, MAX_low = 0;
	uint16_t MIN_high = 4095, MAX_high = 0;

	for(int i = start_sample;i < (start_sample + SAMPLES/2); i+=2){
		if(ADC_RAW[i] > 4094){//проверяем слаботочный вход на "зашкаливание"
			//зашкалило - используемый высокотоковый сэмпл, если выше шума на входе
			HIGH_Amp.Summ += (float)ADC_RAW[i+1];
			HIGH_Amp.Count++;
		}
		else{//не зашкалило - используем слаботочный сэмпл
			LOW_Amp.Summ += (float)ADC_RAW[i];
			LOW_Amp.Count++;
		}

		//ищем минимальные и максимальные значения для каналов
		if(MIN_low > ADC_RAW[i])
			MIN_low = ADC_RAW[i];
		if(MAX_low < ADC_RAW[i] && ADC_RAW[i] < 4096)
			MAX_low = ADC_RAW[i];
		if(MIN_high > ADC_RAW[i+1])
			MIN_high = ADC_RAW[i+1];
		if(MAX_high < ADC_RAW[i+1] && ADC_RAW[i+1] > (uint16_t)(HIGH_Amp.Zero_Level))
			MAX_high = ADC_RAW[i+1];
	}
	HIGH_Amp.Summ -=  (uint32_t)(HIGH_Amp.Zero_Level * (float)HIGH_Amp.Count);
	LOW_Amp.Summ -= (uint32_t)(LOW_Amp.Zero_Level * (float)LOW_Amp.Count);
	lowmax = LOW_Amp.Count;
	highmax = HIGH_Amp.Count;

	ACT_current = (float)((float)LOW_Amp.Summ/LOW_C + (float)HIGH_Amp.Summ/HIGH_C) * V_REF / (SAMPLES/4) / R_SHUNT / 4095.f;//mV

	if(MIN_low/1000.f < MIN_high)
		MIN_current = MIN_low * LOW_PR;
	else
		MIN_current = MIN_high * HIGH_PR;

	if(MAX_low == 4095)
		MAX_current = MAX_high * HIGH_PR;
	else
		MAX_current = MAX_low * LOW_PR;
}

void Display_Data(void)
{
	Time_seconds++;
	Clock_Seconds++;
	if(Clock_Seconds%60 == 0){
		Clock_Minutes++;
		Clock_Seconds = 0;
		if(Clock_Minutes%60 ==0){
			Clock_Hours++;
			Clock_Minutes = 0;
			if(Clock_Hours%24 == 0){
				Clock_Days++;
				Clock_Hours = 0;
			}
		}
	}
	mAh_consumed += ACT_current;

	uint16_t Vbat = 0;
	LL_GPIO_SetOutputPin(Vbat_ON_GPIO_Port, Vbat_ON_Pin);
	LL_ADC_REG_StartConversionSWStart(ADC2);
	while(LL_ADC_IsActiveFlag_EOS(ADC2)==0);
	Vbat = LL_ADC_REG_ReadConversionData12(ADC2);
	Vbat = (uint16_t)(__LL_ADC_CALC_DATA_TO_VOLTAGE((uint32_t) V_REF, Vbat, LL_ADC_RESOLUTION_12B) / RES_DEVIDER);
	LL_GPIO_ResetOutputPin(Vbat_ON_GPIO_Port, Vbat_ON_Pin);

	if(DSP_flag != 0){
		SSD1306_Fill(SSD1306_COLOR_BLACK);
		SSD1306_GotoXY(0,0);
		char time_str[25] = {'\0'};
		sprintf(time_str,"D:%1i H:%02i M:%02i S:%02i", (int)Clock_Days,(int)Clock_Hours,(int)Clock_Minutes,(int)Clock_Seconds);
		SSD1306_Puts(time_str,&Font_7x10,SSD1306_COLOR_WHITE);

		SSD1306_GotoXY(0,16);
		char Curr_str[20] = {'\0'};
		sprintf(Curr_str,"%.6f mA",ACT_current);
		SSD1306_Puts(Curr_str,&Font_7x10,SSD1306_COLOR_WHITE);

		SSD1306_GotoXY(0,26);
		char mAh_str[20]  = {'\0'};
		SSD1306_GotoXY(0,26);
		sprintf(mAh_str,"%.6f aver.",(float)(mAh_consumed/Time_seconds));
		SSD1306_Puts(mAh_str,&Font_7x10,SSD1306_COLOR_WHITE);

		SSD1306_GotoXY(0,36);
		char mAh_con_str[20]  = {'\0'};
		sprintf(mAh_con_str,"%.6f mAh",(float)(mAh_consumed/3600));
		SSD1306_Puts(mAh_con_str,&Font_7x10,SSD1306_COLOR_WHITE);

		SSD1306_GotoXY(0,48);
		char max_curr_str[20]  = {'\0'};
		sprintf(max_curr_str,"%.6f mA",(float)(MAX_current));
		SSD1306_Puts(max_curr_str,&Font_7x10,SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
	}

	if(MAX_current_ALLTime < MAX_current)
		MAX_current_ALLTime = MAX_current;

	char USB_string[100];
	sprintf(USB_string,"start %u %u %u %u %u %u %u stop\r\n",
			(unsigned int)Time_seconds,				//seconds
			(unsigned int)(MIN_current*1000000.f),	//nA
			(unsigned int)(ACT_current*1000000.f),	//nA
			(unsigned int)((float)mAh_consumed/3.6),		//mkAh
			(unsigned int)(MAX_current*1000000.f),	//nA
			(unsigned int)(MAX_current_ALLTime*1000000.f),//nA
			(unsigned int)Vbat);						//mV
	CDC_Transmit_FS((unsigned char*)&USB_string,(uint16_t)strlen(USB_string));
	LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
}

void Amp_Calibration(void)
{
	DSP_flag = SSD1306_Init();
	if(DSP_flag != 0){
		SSD1306_GotoXY(22,32);
		SSD1306_Puts("Calibration",&Font_7x10,SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
	}

	uint32_t HIGH_noise = 0;
	uint32_t LOW_noise = 0;
	for(int i = 0; i<2;i++){
		while(!FT_flag){
		}
		for(int j = 0;j<SAMPLES;j+=2){
			LOW_noise += ADC_RAW[j];
			HIGH_noise += ADC_RAW[j+1];
		}
		FT_flag = 0;
	}
	HIGH_Amp.Zero_Level = (float)((float)HIGH_noise/(float)SAMPLES);
	LOW_Amp.Zero_Level 	= (float)((float)LOW_noise/(float)SAMPLES);
}

void Device_CFG(void)
{
	LL_TIM_DisableCounter(TIM1);
	uint32_t calib_wait = 0;
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_ConfigAddresses(DMA1,LL_DMA_CHANNEL_1,LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
												(uint32_t)&ADC_RAW, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, SAMPLES);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
	LL_ADC_REG_StartConversionExtTrig(ADC1,LL_ADC_REG_TRIG_EXT_RISING);

	LL_ADC_Enable(ADC1);
	calib_wait = ((LL_ADC_DELAY_ENABLE_CALIB_ADC_CYCLES * 32) >> 1);
	while(calib_wait != 0){
		calib_wait--;
	}
	LL_ADC_StartCalibration(ADC1);
	while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0) {
	}
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM1);

	LL_ADC_Enable(ADC2);
	calib_wait = ((LL_ADC_DELAY_ENABLE_CALIB_ADC_CYCLES * 32) >> 1);
	while(calib_wait != 0){
		calib_wait--;
	}
	LL_ADC_StartCalibration(ADC2);
	while (LL_ADC_IsCalibrationOnGoing(ADC2) != 0) {
	}

	LL_ADC_Enable(ADC1);
	LL_mDelay(1);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
	//Amp_Calibration();

	Time_seconds = 0;
	FT_flag = HT_flag = 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
