/**
 *		||					____	_ __													 
 * +------+			 / __ )(_) /_______________ _____  ___ 
 * | 0xBC |			/ __	/ / __/ ___/ ___/ __ `/_	/ / _ \
 * +------+		 / /_/ / / /_/ /__/ /  / /_/ / / /_/	__/
 *	||	||		/_____/_/\__/\___/_/	 \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * adc.c - Analog Digital Conversion
 *
 * TODO: Describe functionality.
 *
 * Sample time: According to the formula in the stm32 product manual
 *							page 69, with a Ts of 28.5 samples, 12-bit, and ADC@12
 *							the highest impedance to use is 25.2kOhm.
 *
 *	This file includes the code to read the proximity sensors in the ADC and log their data
 */
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "adc.h"
#include "pm.h"
#include "nvicconf.h"
#include "imu.h"
#include "log.h"
#include "debug.h"

#ifdef ADC_OUTPUT_RAW_DATA
#include "uart.h"
#include "acc.h"
#endif

// PORT A GPIOs for crazyflie 2.0
#define GPIO_A0_PIN GPIO_Pin_2
#define GPIO_A1_PIN GPIO_Pin_3
#define GPIO_A2_PIN GPIO_Pin_5
#define GPIO_A3_PIN GPIO_Pin_6
#define GPIO_A4_PIN GPIO_Pin_7


// CHANNELS
#define NBR_OF_ADC_CHANNELS		1
#define CH_A0	ADC_Channel_2
#define CH_A1	ADC_Channel_3
#define CH_A2	ADC_Channel_5
#define CH_A3	ADC_Channel_6
#define CH_A4	ADC_Channel_7

#define CH_VREF	ADC_Channel_17
#define CH_TEMP	ADC_Channel_16

static bool isInit = false;

xQueueHandle adcQueue;

// logging info for prox sensor
volatile uint16_t ADCBuffer[NBR_OF_ADC_CHANNELS];

volatile uint16_t prox1_value;

LOG_GROUP_START(adc)
//LOG_ADD(LOG_UINT16, prox1, &prox1_value)
LOG_ADD(LOG_UINT16, prox1, &ADC1->DR)
LOG_GROUP_STOP(adc)


/**
 * Initialize the ADC to read proximity sensors.
 * Most of this code was taken and adapted from 
 * http://survivalengineer.blogspot.com/2013/03/stm32f4-and-most-of-what-you-ever.html
 */
void adcInit(void)
{
	// Define init structures 
	ADC_InitTypeDef			ADC_InitStructure;
	ADC_CommonInitTypeDef	ADC_CommonInitStructure;
	NVIC_InitTypeDef		NVIC_InitStructure;
	DMA_InitTypeDef			DMA_InitStructure;
	GPIO_InitTypeDef		GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;

	// Enable timer (timer runs at 21 MHz)
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_Period = 1999;
//	TIM_TimeBaseStructure.TIM_Prescaler = 17999;
//	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
//	TIM_SelectOutputTrigger(TIM2,TIM_TRGOSource_Update);
//	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
//	NVIC_Init(&NVIC_InitStructure);

	// Enable clock on DMA1 & GPIOA 
	// Enable DMA2, thats where ADC is hooked on -> see Table 43 (RM00090) 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2, ENABLE);

	// Initialise GPIOs
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin	 = GPIO_A0_PIN | GPIO_A1_PIN | GPIO_A2_PIN |
								   GPIO_A3_PIN | GPIO_A4_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Initialise DMA, ADC1 is connected to DMA2 Channel 0 Stream 0
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_BufferSize = NBR_OF_ADC_CHANNELS;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; // no FIFO 
	DMA_InitStructure.DMA_FIFOThreshold = 0;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; // circular buffer 
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; // high priority 
	// config of memory 
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ADCBuffer; // target addr 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // 16 bit 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; // source address
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	// reset ADC configs and structures
	ADC_StructInit(&ADC_InitStructure);
	ADC_CommonStructInit(&ADC_CommonInitStructure);
	ADC_Cmd(ADC1, DISABLE);
	ADC_DeInit();

	// init ADC clock 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// init ADC 
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// ADC1 Init: this is mostly done with ADC1->CR 
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
//	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
//	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = 0;
//	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = NBR_OF_ADC_CHANNELS;
	ADC_Init(ADC1, &ADC_InitStructure);

	// Enable Vref & Temperature channel 
	ADC_TempSensorVrefintCmd(ENABLE);

	// Configure channels 
	ADC_RegularChannelConfig(ADC1, CH_A0, 1, ADC_SampleTime_480Cycles);			
//	ADC_RegularChannelConfig(ADC1, CH_A1, 2, ADC_SampleTime_480Cycles); 
//	ADC_RegularChannelConfig(ADC1, CH_A2, 3, ADC_SampleTime_480Cycles);
//	ADC_RegularChannelConfig(ADC1, CH_A3, 4, ADC_SampleTime_480Cycles);
//	ADC_RegularChannelConfig(ADC1, CH_A4, 5, ADC_SampleTime_480Cycles);

	// Enable DMA request after last transfer (Single-ADC mode) 
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	// Start Timer 2 (and begin conversions)
//	TIM_Cmd(TIM2, ENABLE);
}

/*
void __old_adcInit(void)
{
	DEBUG_PRINT("adcInit() start\n");

	if(isInit)
		return;

	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable TIM2, GPIOA and ADC1 clock
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//GPIO Init Strcuture for proximity sensor
	GPIO_InitStructure.GPIO_Pin = GPIO_PROX; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	//Timer configuration
//	TIM_TimeBaseStructure.TIM_Period = ADC_TRIG_PERIOD;
//	TIM_TimeBaseStructure.TIM_Prescaler = ADC_TRIG_PRESCALE;
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
//
//	// TIM2 channel2 configuration in PWM mode
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_Pulse = 1;
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
//	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
//	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
//	// Halt timer 2 during debug halt.
//	DBGMCU_Config(DBGMCU_TIM2_STOP, ENABLE);

//	adcDmaInit();

	// ADC Common Init
	ADC_DeInit();
//	ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult;
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
//	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_2;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// ADC1 configuration
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
//	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = NBR_OF_ADC_CHANNELS;
	ADC_Init(ADC1, &ADC_InitStructure);

	// ADC1 channel sequence
	ADC_RegularChannelConfig(ADC1, CH_PROX, 1, ADC_SampleTime_28Cycles);

	ADC_ContinuousModeCmd(ADC1, ENABLE);

	// ADC2 configuration
//	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
//	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
//	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
//	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//	ADC_InitStructure.ADC_NbrOfConversion = NBR_OF_ADC_CHANNELS;
//	ADC_Init(ADC2, &ADC_InitStructure);
//
//	// ADC2 channel sequence
//	ADC_RegularChannelConfig(ADC2, CH_PROX, 1, ADC_SampleTime_28Cycles);

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	// Calibrate ADC1
//	ADC_ResetCalibration(ADC1);
//	while(ADC_GetResetCalibrationStatus(ADC1));
//	ADC_StartCalibration(ADC1);
//	while(ADC_GetCalibrationStatus(ADC1));

	// Enable ADC1 external trigger
//	ADC_ExternalTrigConvCmd(ADC1, ENABLE);
//	ADC_TempSensorVrefintCmd(ENABLE);

	// Enable ADC2
//	ADC_Cmd(ADC2, ENABLE);
	// Calibrate ADC2
//	ADC_ResetCalibration(ADC2);
//	while(ADC_GetResetCalibrationStatus(ADC2));
//	ADC_StartCalibration(ADC2);
//	while(ADC_GetCalibrationStatus(ADC2));

	// Enable ADC2 external trigger
//	ADC_ExternalTrigConvCmd(ADC2, ENABLE);

	// Enable the DMA1 channel1 Interrupt
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_ADC_PRI;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

//	adcQueue = xQueueCreate(1, sizeof(AdcGroup*));
//
//	xTaskCreate(adcTask, (const signed char * const)"ADC",
//							configMINIMAL_STACK_SIZE, NULL, 3, NULL);

	isInit = true;
	DEBUG_PRINT("adcInit() complete\n");
}
*/

bool adcTest(void)
{
	return isInit;
}

//float adcConvertToVoltageFloat(uint16_t v, uint16_t vref)
//{
//	return (v / (vref / ADC_INTERNAL_VREF));
//}

//void adcDmaStart(void)
//{
//	// Enable the Transfer Complete and Half Transfer Interrupt
//	DMA_ITConfig(DMA1_Stream1, DMA_IT_TC | DMA_IT_HT, ENABLE);
//	// Enable ADC1 DMA
//	ADC_DMACmd(ADC1, ENABLE);
//	// TIM2 counter enable
////	TIM_Cmd(TIM2, ENABLE);
//}

//void adcDmaStop(void)
//{
////	TIM_Cmd(TIM2, DISABLE);
//}

// ADC Interrupt handler, called from the DMA1_Channel1_IRQHandler in nvic.c
void adcInterruptHandler(void)
{
	/*portBASE_TYPE xHigherPriorityTaskWoken;
	AdcGroup* adcBuffer;

	if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_HTIF1))
	{
		DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_HTIF1);
		adcBuffer = (AdcGroup*)&adcValues[0];
//		xQueueSendFromISR(adcQueue, &adcBuffer, &xHigherPriorityTaskWoken);
	}
	if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1))
	{
		DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
		adcBuffer = (AdcGroup*)&adcValues[ADC_MEAN_SIZE];
//		xQueueSendFromISR(adcQueue, &adcBuffer, &xHigherPriorityTaskWoken);
	}*/
	DEBUG_PRINT("adcInterruptHandler Finished\n");
}

void adcTask(void *param)
{
	AdcGroup* adcRawValues;
	AdcGroup adcValues;

	vTaskSetApplicationTaskTag(0, (void*)TASK_ADC_ID_NBR);
	vTaskDelay(1000);

	adcDmaStart();

	while(1)
	{
		xQueueReceive(adcQueue, &adcRawValues, portMAX_DELAY);
		adcDecimate(adcRawValues, &adcValues);	// 10% CPU
		proxSensorUpdate(&adcValues);

#ifdef ADC_OUTPUT_RAW_DATA
		uartSendDataDma(sizeof(AdcGroup)*ADC_MEAN_SIZE, (uint8_t*)adcRawValues);
#endif
	}
}


//void adcDmaInit(void)
//{
//	DMA_InitTypeDef DMA_InitStructure;
//
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
//
//	// DMA channel1 configuration
//	DMA_DeInit(DMA1_Stream1);
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
////	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&adcValues;
//	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&prox1_value;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
////	DMA_InitStructure.DMA_BufferSize = NBR_OF_ADC_CHANNELS * (ADC_MEAN_SIZE * 2);
//	DMA_InitStructure.DMA_BufferSize = 1;
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
////	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//	DMA_Init(DMA1_Stream1, &DMA_InitStructure);
//	// Enable DMA channel1
//	DMA_Cmd(DMA1_Stream1, ENABLE);
//}

/**
 * Decimates the adc samples after oversampling
 */
//void adcDecimate(AdcGroup* oversampled, AdcGroup* decimated)
//{
//	uint32_t i, j;
//	uint32_t sum;
//	uint32_t sumVref;
//	AdcGroup* adcIterator;
//	AdcPair *adcOversampledPair;
//	AdcPair *adcDecimatedPair;
//
//	// Compute sums and decimate each channel
//	adcDecimatedPair = (AdcPair*)decimated;
//	for (i = 0; i < NBR_OF_ADC_CHANNELS; i++)
//	{
//		adcIterator = oversampled;
//		sum = 0;
//		sumVref = 0;
//		for (j = 0; j < ADC_MEAN_SIZE; j++)
//		{
//			adcOversampledPair = &((AdcPair*)adcIterator)[i];
//			sum += adcOversampledPair->val;
//			sumVref += adcOversampledPair->vref;
//			adcIterator++;
//		}
//		// Decimate
//		adcDecimatedPair->val = sum / ADC_DECIMATE_DIVEDEND;
//		adcDecimatedPair->vref = sumVref / ADC_DECIMATE_DIVEDEND;
//		adcDecimatedPair++;
//	}
//}
