/**
 *    ||           ____  _ __                           
 * +------+       / __ )(_) /_______________ _____  ___ 
 * | 0xBC |      / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+     / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||     /_____/_/\__/\___/_/   \__,_/ /___/\___/
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
#define NBR_OF_ADC_CHANNELS		5
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

LOG_GROUP_START(adc)
LOG_ADD(LOG_UINT16, A0, &ADCBuffer[0])
LOG_ADD(LOG_UINT16, A1, &ADCBuffer[1])
LOG_ADD(LOG_UINT16, A2, &ADCBuffer[2])
LOG_ADD(LOG_UINT16, A3, &ADCBuffer[3])
LOG_ADD(LOG_UINT16, A4, &ADCBuffer[4])
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

	if (isInit)
		return;

	// Enable timer to trigger ADC conversion
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = ADC_TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler = ADC_TIM_PRESCALE;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	// configure timer interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_Init(&NVIC_InitStructure);

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
	GPIO_Init(GPIOA, &GPIO_InitStructure);

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
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = NBR_OF_ADC_CHANNELS;
	ADC_Init(ADC1, &ADC_InitStructure);

	// Enable Vref & Temperature channel 
	ADC_TempSensorVrefintCmd(ENABLE);

	// Configure channels 
	ADC_RegularChannelConfig(ADC1, CH_A0, 1, ADC_SampleTime_480Cycles);			
	ADC_RegularChannelConfig(ADC1, CH_A1, 2, ADC_SampleTime_480Cycles); 
	ADC_RegularChannelConfig(ADC1, CH_A2, 3, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, CH_A3, 4, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, CH_A4, 5, ADC_SampleTime_480Cycles);

	// Enable DMA request after last transfer (Single-ADC mode) 
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	// Start Timer 3 (and begin conversions)
	TIM_Cmd(TIM3, ENABLE);

	isInit = true;
}

uint16_t getADCValue(int channel)
{
	if (channel >= NBR_OF_ADC_CHANNELS)
		return 0;
	
	return ADCBuffer[channel];
}

/*
void __old_adcInit(void)
{
	DEBUG_PRINT("adcInit() start\n");

	// Enable ADC2 external trigger
	ADC_ExternalTrigConvCmd(ADC2, ENABLE);

	// Enable the DMA1 channel1 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_ADC_PRI;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	adcQueue = xQueueCreate(1, sizeof(AdcGroup*));

	xTaskCreate(adcTask, (const signed char * const)"ADC",
							configMINIMAL_STACK_SIZE, NULL, 3, NULL);

	isInit = true;
	DEBUG_PRINT("adcInit() complete\n");
}
*/

bool adcTest(void)
{
	return isInit;
}

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
//	AdcGroup* adcRawValues;
//	AdcGroup adcValues;
//
//	vTaskSetApplicationTaskTag(0, (void*)TASK_ADC_ID_NBR);
//	vTaskDelay(1000);
//
//	adcDmaStart();
//
//	while(1)
//	{
//		xQueueReceive(adcQueue, &adcRawValues, portMAX_DELAY);
//		adcDecimate(adcRawValues, &adcValues);	// 10% CPU
//		proxSensorUpdate(&adcValues);
//	}
}
