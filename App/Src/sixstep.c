/*
 * sixstep.c
 *
 *  Created on: Nov 24, 2018
 *      Author: Robin
 */

#include "sixstep.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"
#include "dac.h"

extern void Error_Handler(void);

#define SIXSTEP_MODE_ALIGN  0x01
#define SIXSTEP_MODE_RAMPUP 0x02
#define SIXSTEP_MODE_CLOSED 0x04
#define SIXSTEP_BEMF_SAMPLETIME ADC_SAMPLETIME_61CYCLES_5
#define SIXSTEP_ADC_VBUS_FILTER_DEPTH 10
#define SIXSTEP_ADC_TEMP_FILTER_DEPTH 10
#define SIXSTEP_ADC_CURRENT_FILTER_DEPTH 10

static int is_running = 0;
static int current_mode = SIXSTEP_MODE_RAMPUP;
static int current_step = 0;
static volatile unsigned long int adc_temp = 0;
static volatile unsigned long int adc_vbus = 0;
static volatile unsigned long int adc_current = 0;
static volatile int adc_ovf = 0;
static volatile int adc_udf = 0;

static int adc_bemf_current_channel = ADC_CHANNEL_9;
static int adc_bemf_channel[3] = {
    ADC_CHANNEL_9,
    ADC_CHANNEL_11,
    ADC_CHANNEL_15
};

static int adc_inst_current_channel = 0;
static int adc_inst_channel[3] = {
    ADC_CHANNEL_7,  // current sense
    ADC_CHANNEL_8,  // temperature
    ADC_CHANNEL_2   // vbus
};
static int adc_inst_sampletime[3] = {
    ADC_SAMPLETIME_1CYCLE_5,
    ADC_SAMPLETIME_61CYCLES_5,
    ADC_SAMPLETIME_61CYCLES_5
};

static int adc_filter_vbus_i = 0;
static int adc_filter_temp_i = 0;
static int adc_filter_current_i = 0;

static uint16_t adc_filter_vbus[SIXSTEP_ADC_VBUS_FILTER_DEPTH] = {0};
static uint16_t adc_filter_temp[SIXSTEP_ADC_TEMP_FILTER_DEPTH] = {0};
static uint16_t adc_filter_current[SIXSTEP_ADC_CURRENT_FILTER_DEPTH] = {0};

#define PWM_VAL 600
#define MOTOR_POLES 7

void SixStep_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfig.Offset = 0;
  sConfig.OffsetNumber = 0;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

void SixStep_StartMotor(void)
{
  if (is_running == 1) {
    return;
  }

  /* Start Output PWM */
  HAL_TIM_Base_Start_IT(&htim1);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  /* Start Current Reference DAC */
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1200); /* 500mA current limit */

  SixStep_SetTimerSpeed(1000);
  SixStep_Commutate(5);
  SixStep_StartADC();

  /* Start Commutation Timer */
  HAL_TIM_Base_Start_IT(&htim2);

  is_running = 1;
}

void SixStep_StopMotor(void)
{
  /* Stop commutation timer */
  HAL_TIM_Base_Stop_IT(&htim2);

  /* Stop ADC */
  HAL_ADC_Stop_IT(&hadc1);

  /* Clear and deactivate PWM channels */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

  HAL_TIM_Base_Stop_IT(&htim1);

  /* Pull all input pins on L6230 LOW */
  HAL_GPIO_WritePin(L6230_EN_CH1_GPIO_Port, L6230_EN_CH1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L6230_EN_CH2_GPIO_Port, L6230_EN_CH2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L6230_EN_CH3_GPIO_Port, L6230_EN_CH3_Pin, GPIO_PIN_RESET);

  is_running = 0;
}

void SixStep_StartADC(void)
{
  HAL_ADC_Start_IT(&hadc1);
}

void SixStep_CommutationInterrupt(void)
{
  if (current_mode != SIXSTEP_MODE_RAMPUP) {
    return;
  }

  SixStep_Commutate(current_step);

  current_step++;
  if (current_step == 6) {
    current_step = 0;
  }
}

void SixStep_ADCInterrupt(void)
{
  uint16_t next_channel;
  uint32_t val = HAL_ADC_GetValue(&hadc1);

  /* Up-counting, so we have the latest BEMF reading */
  if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)) {
    /* TODO: delay for de-magnetization */

    if (current_mode == SIXSTEP_MODE_RAMPUP) {
      /* check current speed and count z-crossings */
    }
    else if (current_mode == SIXSTEP_MODE_CLOSED)
    {
      /* TODO: process BEMF reading here */
    }

    // next read will be instrumentation channel
    next_channel = adc_inst_channel[adc_inst_current_channel];

    adc_ovf = 1;
  }
  /* Down-counting, cycling through the instrumentation readings */
  else {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

    /* TODO: process instrumentation reading here */
    switch (adc_inst_current_channel) {
    case 0:
    {
      adc_filter_current[adc_filter_current_i++] = val;
      if (adc_filter_current_i == SIXSTEP_ADC_CURRENT_FILTER_DEPTH) {
        uint32_t avg = 0;
        for (int i = 0; i < SIXSTEP_ADC_CURRENT_FILTER_DEPTH; i++) {
          avg += adc_filter_current[i];
        }
        avg /= SIXSTEP_ADC_CURRENT_FILTER_DEPTH;
        adc_current = avg;
        adc_filter_current_i = 0;
      }
      break;
    }
    case 1:
    {
      adc_filter_temp[adc_filter_temp_i++] = val;
      if (adc_filter_temp_i == SIXSTEP_ADC_TEMP_FILTER_DEPTH) {
        uint32_t avg = 0;
        for (int i = 0; i < SIXSTEP_ADC_TEMP_FILTER_DEPTH; i++) {
          avg += adc_filter_temp[i];
        }
        avg /= SIXSTEP_ADC_TEMP_FILTER_DEPTH;
        adc_temp = avg;
        adc_filter_temp_i = 0;
      }
      break;
    }
    case 2:
    {
      adc_filter_vbus[adc_filter_vbus_i++] = val;
      if (adc_filter_vbus_i == SIXSTEP_ADC_VBUS_FILTER_DEPTH) {
        uint32_t avg = 0;
        for (int i = 0; i < SIXSTEP_ADC_VBUS_FILTER_DEPTH; i++) {
          avg += adc_filter_vbus[i];
        }
        avg /= SIXSTEP_ADC_VBUS_FILTER_DEPTH;
        adc_vbus = avg;
        adc_filter_vbus_i = 0;
      }
      break;
    }
    } /* end switch */

    adc_inst_current_channel++;
    if (adc_inst_current_channel >= 3) {
      adc_inst_current_channel = 0;
    }

    // next read will be BEMF channel
    next_channel = adc_bemf_current_channel;
    adc_udf = 1;
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  }

  hadc1.Instance->CR |= ADC_CR_ADSTP;
  while (hadc1.Instance->CR & ADC_CR_ADSTP);
  hadc1.Instance->SQR1 &= ~ADC_SQR1_RK(ADC_SQR2_SQ5, 1);
  hadc1.Instance->SQR1 |= ADC_SQR1_RK(next_channel, 1);
  hadc1.Instance->CR |= ADC_CR_ADSTART;
}

void SixStep_SetTimerSpeed(unsigned int rpm)
{
  unsigned long t = 60000000 / (MOTOR_POLES * rpm * 6);
  __HAL_TIM_SET_AUTORELOAD(&htim2, t);
}

void SixStep_Commutate(int step)
{
  switch (step)
  {
  case 0:
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_VAL);
    HAL_GPIO_WritePin(L6230_EN_CH1_GPIO_Port, L6230_EN_CH1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH2_GPIO_Port, L6230_EN_CH2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH3_GPIO_Port, L6230_EN_CH3_Pin, GPIO_PIN_RESET);
    adc_bemf_current_channel = adc_bemf_channel[1];
    break;
  case 1:
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_VAL);
    HAL_GPIO_WritePin(L6230_EN_CH1_GPIO_Port, L6230_EN_CH1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH3_GPIO_Port, L6230_EN_CH3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH2_GPIO_Port, L6230_EN_CH2_Pin, GPIO_PIN_RESET);
    adc_bemf_current_channel = adc_bemf_channel[2];
    break;
  case 2:
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_VAL);
    HAL_GPIO_WritePin(L6230_EN_CH2_GPIO_Port, L6230_EN_CH2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH3_GPIO_Port, L6230_EN_CH3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH1_GPIO_Port, L6230_EN_CH1_Pin, GPIO_PIN_RESET);
    adc_bemf_current_channel = adc_bemf_channel[2];
    break;
  case 3:
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_VAL);
    HAL_GPIO_WritePin(L6230_EN_CH1_GPIO_Port, L6230_EN_CH1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH2_GPIO_Port, L6230_EN_CH2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH3_GPIO_Port, L6230_EN_CH3_Pin, GPIO_PIN_RESET);
    adc_bemf_current_channel = adc_bemf_channel[0];
    break;
  case 4:
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_VAL);
    HAL_GPIO_WritePin(L6230_EN_CH1_GPIO_Port, L6230_EN_CH1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH3_GPIO_Port, L6230_EN_CH3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH2_GPIO_Port, L6230_EN_CH2_Pin, GPIO_PIN_RESET);
    adc_bemf_current_channel = adc_bemf_channel[0];
    break;
  case 5:
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_VAL);
    HAL_GPIO_WritePin(L6230_EN_CH2_GPIO_Port, L6230_EN_CH2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH3_GPIO_Port, L6230_EN_CH3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH1_GPIO_Port, L6230_EN_CH1_Pin, GPIO_PIN_RESET);
    adc_bemf_current_channel = adc_bemf_channel[1];
    break;
  }
}

void SixStep_SetCurrentLimit(unsigned int current_limit)
{

}
