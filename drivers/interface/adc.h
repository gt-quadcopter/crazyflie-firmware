/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
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
 * adc.h - Analog Digital Conversion header file
 */
#ifndef ADC_H_
#define ADC_H_

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "config.h"

// Common Definitions

/**
 * ADC interrupt handler, which is actually triggered from DMA
 */
void adcInterruptHandler(void);

/**
 * Initialize analog to digital converter. Configures gyro and vref channels.
 * Configures DMA to transfer the result.
 */
void adcInit(void);

/**
 * Returns whether the ADC has been initialized
 */
bool adcTest(void);

#ifdef STM32F4XX
/*************** Crazyflie 2.0 Functions/Options *********************/

/**
 * Get stored value from the last ADC conversion
 */
uint16_t getADCValue(int channel);

#define ADC_N_CHANNELS		5
#define ADC_N_OVERSAMP		8	// number of sample points to average

// 2x buffer length for double buffering
#define ADC_BUFFER_LEN		(2*ADC_N_CHANNELS*ADC_N_OVERSAMP)
//#define ADC_BUFFER_LEN		ADC_N_CHANNELS
#define ADC_SAMPLING_FREQ	100	// in Hz
#define ADC_OVERSAMP_FREQ	(ADC_SAMPLING_FREQ * ADC_N_OVERSAMP)

// 84MHz / 840 = 100KHz, so the tick period is 0.01ms
#define ADC_TIM_PRESCALE	(840-1)
#define ADC_TIM_PERIOD		(((84000000 / (ADC_TIM_PRESCALE+1)) / ADC_OVERSAMP_FREQ)-1)

#else
/***************** Crazyflie 1 Functions/Options ********************/

/******** Defines ********/

/**
 * \def ADC_MEAN_SIZE
 * Number of samples used in the mean value calculation.
 * Mean size should be evenly dividable by decimation bits.
 */
#define ADC_DECIMATE_TO_BITS  12
#define ADC_MEAN_SIZE         8

#define ADC_RESOLUTION        12
#define ADC_DECIMATE_DIVEDEND (ADC_MEAN_SIZE / (1 << (ADC_DECIMATE_TO_BITS - ADC_RESOLUTION)))

#if ADC_DECIMATE_TO_BITS < ADC_RESOLUTION
#  error "ADC_DECIMATE_TO_BITS must be bigger or equal to ADC_RESOLUTION"
#endif

#define ADC_SAMPLING_FREQ      100
#define ADC_OVERSAMPLING_FREQ  (ADC_SAMPLING_FREQ * ADC_MEAN_SIZE)

#define ADC_TRIG_PRESCALE       1
#define ADC_TRIG_PRESCALE_FREQ  (72000000 / (ADC_TRIG_PRESCALE + 1))
#define ADC_TRIG_PERIOD         (ADC_TRIG_PRESCALE_FREQ / (ADC_OVERSAMPLING_FREQ))

#define ADC_INTERNAL_VREF   1.20


/******** Types ********/

typedef struct __attribute__((packed))
{
  uint16_t vref;
  uint16_t val;
} AdcPair;

typedef struct __attribute__((packed))
{
  AdcPair vprox1;
} AdcGroup;

typedef struct
{
  uint16_t vbat;
  uint16_t vbatVref;
} AdcDeciGroup;

/*** Public interface ***/

/**
 * Converts a 12 bit ADC value to battery voltage
 * @param vbat  12 bit adc value
 * @param vref  12 bit adc value of the internal voltage
 *              reference, 1.2V
 *
 * @return The voltage in a float value
 */
float adcConvertToVoltageFloat(uint16_t v, uint16_t vref);

/**
 * Starts converting ADC samples by activating the DMA.
 */
void adcDmaStart(void);

/**
 * Stop converting ADC samples.
 */
void adcDmaStop(void);

/**
 * ADC task
 */
void adcTask(void *param);

#endif /* STM32F4XX */
#endif /* ADC_H_ */
