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

// PORT A GPIOs for crazyflie 2.0
#define GPIO_A0_PIN GPIO_Pin_2
#define GPIO_A1_PIN GPIO_Pin_3
#define GPIO_A2_PIN GPIO_Pin_5
#define GPIO_A3_PIN GPIO_Pin_6
#define GPIO_A4_PIN GPIO_Pin_7


// ADC CHANNELS
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
volatile uint16_t ADCBuffer[ADC_BUFFER_LEN];

float ADCFloats[ADC_N_CHANNELS];

LOG_GROUP_START(adc)
LOG_ADD(LOG_UINT16, A0, &ADCBuffer[0])
LOG_ADD(LOG_UINT16, A1, &ADCBuffer[1])
LOG_ADD(LOG_UINT16, A2, &ADCBuffer[2])
LOG_ADD(LOG_UINT16, A3, &ADCBuffer[3])
LOG_ADD(LOG_UINT16, A4, &ADCBuffer[4])

LOG_ADD(LOG_FLOAT, A0_f, &ADCFloats[0])
LOG_ADD(LOG_FLOAT, A1_f, &ADCFloats[1])
LOG_ADD(LOG_FLOAT, A2_f, &ADCFloats[2])
LOG_ADD(LOG_FLOAT, A3_f, &ADCFloats[3])
LOG_ADD(LOG_FLOAT, A4_f, &ADCFloats[4])
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

	// Enable clock on DMA2 & GPIOA 
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
	DMA_InitStructure.DMA_BufferSize = ADC_BUFFER_LEN;
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

	// comfig DMA interrupts and start DMA
	DMA_ITConfig(DMA2_Stream0, DMA_IT_HTIF0, ENABLE);
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TCIF0, ENABLE);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	// configure DMA interrupt
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_ADC_PRI;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

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
	ADC_InitStructure.ADC_NbrOfConversion = ADC_N_CHANNELS;
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
	
#ifdef ADC_GPIOC12_TIMING_DEBUG
	// enable GPIO C12 for timing the averaging
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode =	GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed =	GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType =	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd =	GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

	// Start Timer 3 (and begin conversions)
	TIM_Cmd(TIM3, ENABLE);

//	adcQueue = xQueueCreate(1, sizeof(uint16_t*));

//	xTaskCreate(adcTask, (const signed char * const)"ADC",
//							configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	isInit = true;
}

uint16_t getADCValue(int channel)
{
	if (channel >= ADC_N_CHANNELS)
		return 0;
	
	return ADCBuffer[channel];
}

float getADCValue_f(int channel)
{
	if (channel >= ADC_N_CHANNELS)
		return 0;
	return ADCFloats[channel];
}

/**
 * ADC Interrupt handler, called from the DMA2_Stream0_IRQHandler in nvic.c
 * Because I couldn't get the OS task/queue stuff to work, the oversampling decimation
 * just happens in this IRQ.
 *
 * According to timings measured by toggling GPIOC Pin 12 at the beginning and
 * end of the IRQ, this function takes approximately 11.4us to average 10 samples
 * for each of the 5 channels.
 */
void adcInterruptHandler(void)
{
	int i, j;
	uint16_t *adcValues;
	uint32_t totals[ADC_N_CHANNELS];

#ifdef ADC_GPIOC12_TIMING_DEBUG
	// set C12 for timing analysis
	GPIO_SetBits(GPIOC, GPIO_Pin_12);
#endif

	// initialize totals to 0
	for (i = 0; i < ADC_N_CHANNELS; i++)
		totals[i] = 0;

	// clear DMA IT flag and get address for which half of the buffer to use
	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0))
	{
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);
		adcValues = (uint16_t*)&ADCBuffer[0];
	}
	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
	{
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
		adcValues = (uint16_t*)&ADCBuffer[ADC_BUFFER_LEN / 2];
	}

	// calculate totals
	for (j = 0; j < ADC_N_OVERSAMP; j++)
		for (i = 0; i < ADC_N_CHANNELS; i++)
			totals[i] += adcValues[ADC_N_CHANNELS*j + i];

	// calculate averages
	for (i = 0; i < ADC_N_CHANNELS; i++)
		ADCFloats[i] = (1.0f*totals[i]) / (4095.0f*ADC_N_OVERSAMP);

#ifdef ADC_GPIOC12_TIMING_DEBUG
	GPIO_ResetBits(GPIOC, GPIO_Pin_12);
#endif
}

//void adcTask(void *param)
//{
//	uint16_t *values;
//	uint32_t totals[ADC_N_CHANNELS];
//	int i, j;
//	memset(totals, 0, ADC_N_CHANNELS*sizeof(uint32_t));
//
//	vTaskSetApplicationTaskTag(0, (void*)TASK_ADC_ID_NBR);
//	vTaskDelay(1000);
//
//	while(1)
//	{
//		DEBUG_PRINT("ADC Task Loop\n");
//		xQueueReceive(adcQueue, &values, portMAX_DELAY);
//
//		for (i = 0; i < ADC_N_OVERSAMP; i++)
//		{
//			for (j = 0; j < ADC_N_CHANNELS; j++)
//			{
//				totals[j] += *values;
//				values++;
//			}
//		}
//
//		// scale down the totals for each channel
//		for (i = 0; i < ADC_N_CHANNELS; i++)
//		{
//			ADCFloats[i] = (1.0f*totals[i]) / (4095.0f*ADC_N_OVERSAMP);
//		}
//	}
//}

bool adcTest(void)
{
	return isInit;
}

