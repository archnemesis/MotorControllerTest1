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

#define SIXSTEP_BEMF_SAMPLETIME           ADC_SAMPLETIME_61CYCLES_5
#define SIXSTEP_SPEED_FILTER_DEPTH        16
#define SIXSTEP_ADC_VBUS_FILTER_DEPTH     20
#define SIXSTEP_ADC_TEMP_FILTER_DEPTH     20
#define SIXSTEP_ADC_CURRENT_FILTER_DEPTH  20
#define SIXSTEP_ADC_BEMF_BUFFER_LEN       100
#define SIXSTEP_DEMAG_TIME                10
#define SIXSTEP_BEMF_THRESHOLD_UP         125
#define SIXSTEP_BEMF_THRESHOLD_DOWN       125
#define SIXSTEP_RAMPUP_START_SPEED        10000
#define SIXSTEP_RAMPUP_TARGET             1000
#define SIXSTEP_PI_KP                     4000
#define SIXSTEP_PI_KI                     50
#define SIXSTEP_PI_MIN                    120
#define SIXSTEP_PI_MAX                    3000
#define SIXSTEP_PI_DIV                    4096

static int is_running = 0;
static int current_mode = SIXSTEP_MODE_RAMPUP;
static int current_step = 0;
static int speed_target = 0;
static int pi_i_accum = 0;
static unsigned int last_arr = 0;
static volatile unsigned long int adc_temp = 0;
static volatile unsigned long int adc_vbus = 0;
static volatile unsigned long int adc_current = 0;
static volatile int adc_ovf = 0;
static volatile int adc_udf = 0;
static int adc_last_step = -1;
static int zcross_last_step = -1;
static int demag_counter = 0;
static int demag_time = 0;
static int advance = 0;
static int zcross_counter = 0;
static int zcross_pre_detected = 0;
static int zcross_detected = 0;
static int startup_counter = 0;
static unsigned int last_commutation_time = 0;

static uint32_t comm_arr = 0;

static int adc_bemf_current_channel = ADC_CHANNEL_9;
static int adc_bemf_channel[3] = {
    ADC_CHANNEL_9,
    ADC_CHANNEL_11,
    ADC_CHANNEL_15
};

static int adc_inst_current_channel = 0;
static int adc_inst_channel[3] = {
    ADC_CHANNEL_14,  // current sense
    ADC_CHANNEL_8,  // temperature
    ADC_CHANNEL_2   // vbus
};

static int adc_filter_vbus_i = 0;
static int adc_filter_temp_i = 0;
static int adc_filter_current_i = 0;

static uint16_t adc_filter_vbus[SIXSTEP_ADC_VBUS_FILTER_DEPTH] = {0};
static uint16_t adc_filter_temp[SIXSTEP_ADC_TEMP_FILTER_DEPTH] = {0};
static uint16_t adc_filter_current[SIXSTEP_ADC_CURRENT_FILTER_DEPTH] = {0};

static int speed_filter_i = 0;
static int speed_filter[SIXSTEP_SPEED_FILTER_DEPTH] = {0};
static int speed_filtered = 0;

#define PWM_VAL 550
#define MOTOR_POLES 7

int GetDemagDelay(int speed);

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

  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

void SixStep_StartTimer(void)
{
  HAL_TIM_Base_Start(&htim1);
}

void SixStep_StartMotor(void)
{
  if (is_running == 1) {
    return;
  }

  /* Start Output PWM */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  /* Start Current Reference DAC */
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2000); /* 500mA current limit */

  current_mode = SIXSTEP_MODE_ALIGN;
  SixStep_SetTimerSpeed(500);
  adc_last_step = 0;
  demag_counter = 0;
  current_step = 5;
  speed_target = 1000;
  zcross_counter = 0;
  SixStep_Commutate(5);
  HAL_Delay(500);

  current_mode = SIXSTEP_MODE_RAMPUP;

  /* Start Commutation Timer */
  HAL_TIM_Base_Start_IT(&htim2);

  is_running = 1;
}

void SixStep_StopMotor(void)
{
  is_running = 0;

  /* Stop commutation timer */
  HAL_TIM_Base_Stop_IT(&htim2);

  /* Clear and deactivate PWM channels */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

  /* Pull all input pins on L6230 LOW */
  HAL_GPIO_WritePin(L6230_EN_CH1_GPIO_Port, L6230_EN_CH1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L6230_EN_CH2_GPIO_Port, L6230_EN_CH2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L6230_EN_CH3_GPIO_Port, L6230_EN_CH3_Pin, GPIO_PIN_RESET);
}

void SixStep_StartADC(void)
{
  while (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK);
  HAL_ADC_Start_IT(&hadc1);
}

void SixStep_LowFreqTask(void)
{
  int32_t P = 0;
  int32_t I = 0;
  int32_t i_sum_tmp = 0;
  int32_t output = 0;
  int32_t error = 0;

  if (is_running == 1) {
    if (current_mode == SIXSTEP_MODE_CLOSED) {
      error = speed_target - speed_filtered;
      P = SIXSTEP_PI_KP * error;
      I = SIXSTEP_PI_KI * error;

      i_sum_tmp = pi_i_accum + I;
      pi_i_accum = i_sum_tmp;

      output = HAL_DAC_GetValue(&hdac, DAC_CHANNEL_1) + (P / SIXSTEP_PI_DIV) + (I / SIXSTEP_PI_DIV);

      if (output > SIXSTEP_PI_MAX) {
        output = SIXSTEP_PI_MAX;
      }
      if (output < SIXSTEP_PI_MIN) {
        output = SIXSTEP_PI_MIN;
      }

      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, output);
    }
  }
}

void SixStep_SetTargetSpeed(int rpm)
{
  speed_target = rpm;
}

void SixStep_CommutationInterrupt(void)
{
  long int temp = 0;

  speed_filter[speed_filter_i++] = 60000000 / (MOTOR_POLES * __HAL_TIM_GET_AUTORELOAD(&htim2) * 6);
  if (speed_filter_i == SIXSTEP_SPEED_FILTER_DEPTH) {
    for (int i = 0; i < SIXSTEP_SPEED_FILTER_DEPTH; i++) {
      temp += speed_filter[i];
    }
    temp /= SIXSTEP_SPEED_FILTER_DEPTH;
    speed_filtered = temp;
    speed_filter_i = 0;
  }

  demag_counter = 0;
  zcross_pre_detected = 0;
  demag_time = GetDemagDelay(SixStep_GetFilteredSpeed());
  advance = GetPhaseAdvance(SixStep_GetTimerSpeed());
  current_step++;
  if (current_step == 6) {
    current_step = 0;
  }

  if (current_mode == SIXSTEP_MODE_RAMPUP) {
    if (startup_counter++ == 2) {
      // set to normal startup speed after two quick commutations
      //SixStep_SetTimerSpeed(SIXSTEP_RAMPUP_TARGET);
    }
  }

  SixStep_Commutate(current_step);
}

int GetDemagDelay(int speed)
{
  if (speed < 1000) {
    return 14;
  } else if (speed < 1300) {
    return 13;
  } else if (speed < 1500) {
    return 12;
  } else if (speed < 1800) {
    return 11;
  } else if (speed < 2600) {
    return 10;
  } else if (speed < 3300) {
    return 9;
  } else if (speed < 3650) {
    return 7;
  } else if (speed < 4100) {
    return 6;
  } else if (speed < 4650) {
    return 5;
  } else if (speed < 5400) {
    return 5;
  } else if (speed < 6400) {
    return 4;
  } else if (speed < 7800) {
    return 3;
  } else if (speed < 10000) {
    return 2;
  } else {
    return 1;
  }
}

int GetPhaseAdvance(int speed)
{
  if (speed < 1000) {
    return 0;
  } else if (speed < 1300) {
    return 0;
  } else if (speed < 1500) {
    return 0;
  } else if (speed < 1800) {
    return 1;
  } else if (speed < 2600) {
    return 2;
  } else if (speed < 3300) {
    return 3;
  } else if (speed < 3650) {
    return 4;
  } else if (speed < 4100) {
    return 4;
  } else if (speed < 4650) {
    return 5;
  } else if (speed < 5400) {
    return 5;
  } else if (speed < 6400) {
    return 6;
  } else if (speed < 7800) {
    return 6;
  } else if (speed < 10000) {
    return 7;
  } else {
    return 8;
  }
}

/*
 * @brief Read ADC channels (VBUS, current, temperature, BEMF) and calculate
 * the zero-crossing and commutation times.
 */
void SixStep_ADCInterrupt(void)
{
  uint16_t next_channel;
  uint32_t timer_arr;
  uint32_t timer_cnt;
  uint32_t timer_new_arr;
  uint8_t zcross_missed = 0;
  uint32_t val = HAL_ADC_GetValue(&hadc1);

  /* Down-counting, so we have the latest BEMF reading */
  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)) {
    if (zcross_last_step != current_step) {
      if (zcross_detected == 1) {
        zcross_counter++;
      }
      else {
        zcross_counter = 0;
        zcross_missed = 1;
      }

      if (current_mode == SIXSTEP_MODE_CLOSED) {
        __HAL_TIM_SET_AUTORELOAD(&htim2, 0xFFFF);
      }

      zcross_pre_detected = 0;
      zcross_detected = 0;
      zcross_last_step = current_step;
    }

    if (is_running == 1) {
      if (demag_counter > demag_time) {
        if (adc_last_step != current_step) {
          switch (current_step) {
          case 0:
          case 2:
          case 4:
            if (val < SIXSTEP_BEMF_THRESHOLD_DOWN) {
              zcross_detected = 1;

              if (current_mode == SIXSTEP_MODE_CLOSED) {
                if (demag_counter == (demag_time + 1) && advance > 0) {
                  /**
                   * Zero-crossing was missed so we have to advance the
                   * commutation by about 7.5 degrees.
                   */
                  //HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
                  timer_cnt = __HAL_TIM_GET_COUNTER(&htim2);
                  timer_new_arr = timer_cnt + (timer_cnt / 6);
                  __HAL_TIM_SET_AUTORELOAD(&htim2, timer_new_arr);
                  comm_arr = timer_cnt + (comm_arr / 2);
                }
                else {
                  timer_cnt = __HAL_TIM_GET_COUNTER(&htim2);
                  timer_new_arr = (timer_cnt + (comm_arr / 2));
                  __HAL_TIM_SET_AUTORELOAD(&htim2, timer_new_arr);
                  comm_arr = timer_new_arr;
                }
              }

              HAL_GPIO_WritePin(DEBUG3_GPIO_Port, DEBUG3_Pin, GPIO_PIN_RESET);

              zcross_pre_detected = 0;
              adc_last_step = current_step;
            }
            else {
              zcross_pre_detected++;
            }
            break;
          case 1:
          case 3:
          case 5:
            if (val > SIXSTEP_BEMF_THRESHOLD_UP) {
              zcross_detected = 1;

              if (current_mode == SIXSTEP_MODE_CLOSED) {
                if (demag_counter == (demag_time + 1) && advance > 0) {
                  /**
                   * Zero-crossing was missed so we have to advance the
                   * commutation by about 7.5 degrees.
                   */
                  //HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
                  timer_cnt = __HAL_TIM_GET_COUNTER(&htim2);
                  timer_new_arr = timer_cnt + (timer_cnt / 6);
                  __HAL_TIM_SET_AUTORELOAD(&htim2, timer_new_arr);
                  comm_arr = timer_cnt + (comm_arr / 2);
                }
                else {
                  timer_cnt = __HAL_TIM_GET_COUNTER(&htim2);
                  timer_new_arr = (timer_cnt + (comm_arr / 2));
                  __HAL_TIM_SET_AUTORELOAD(&htim2, timer_new_arr);
                  comm_arr = timer_new_arr;
                }
              }

              HAL_GPIO_WritePin(DEBUG3_GPIO_Port, DEBUG3_Pin, GPIO_PIN_SET);

              zcross_pre_detected = 0;
              adc_last_step = current_step;
            }
            else {
              zcross_pre_detected++;
            }
            break;
          }
        }
      }
      else {
        demag_counter++;
      }
    }

    // next read will be instrumentation channel
    next_channel = adc_inst_channel[adc_inst_current_channel];

    adc_ovf = 1;
  }
  /* Up-counting, cycling through the instrumentation readings */
  else if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)) {
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
  comm_arr = t;
}

unsigned int SixStep_GetTimerSpeed(void)
{
  unsigned long rpm = 60000000 / (MOTOR_POLES * comm_arr * 6);
  return rpm;
}

void SixStep_Commutate(int step)
{
  if (current_mode == SIXSTEP_MODE_RAMPUP) {
    int current_speed = SixStep_GetTimerSpeed();
    comm_arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
    if (current_speed < SIXSTEP_RAMPUP_TARGET) {
      int new_speed = current_speed + 1;
      SixStep_SetTimerSpeed(new_speed);
      zcross_counter = 0;
    }
    else {
      if (zcross_counter > 48) {
        current_mode = SIXSTEP_MODE_CLOSED;
      }
    }
  }

  switch (step)
  {
  case 0:
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_VAL);
    HAL_GPIO_WritePin(L6230_EN_CH3_GPIO_Port, L6230_EN_CH3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(L6230_EN_CH1_GPIO_Port, L6230_EN_CH1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH2_GPIO_Port, L6230_EN_CH2_Pin, GPIO_PIN_SET);
    adc_bemf_current_channel = adc_bemf_channel[2];
    break;
  case 1:
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_VAL);
    HAL_GPIO_WritePin(L6230_EN_CH2_GPIO_Port, L6230_EN_CH2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(L6230_EN_CH1_GPIO_Port, L6230_EN_CH1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH3_GPIO_Port, L6230_EN_CH3_Pin, GPIO_PIN_SET);
    adc_bemf_current_channel = adc_bemf_channel[1];
    break;
  case 2:
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_VAL);
    HAL_GPIO_WritePin(L6230_EN_CH1_GPIO_Port, L6230_EN_CH1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(L6230_EN_CH2_GPIO_Port, L6230_EN_CH2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH3_GPIO_Port, L6230_EN_CH3_Pin, GPIO_PIN_SET);
    adc_bemf_current_channel = adc_bemf_channel[0];
    break;
  case 3:
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_VAL);
    HAL_GPIO_WritePin(L6230_EN_CH3_GPIO_Port, L6230_EN_CH3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(L6230_EN_CH1_GPIO_Port, L6230_EN_CH1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH2_GPIO_Port, L6230_EN_CH2_Pin, GPIO_PIN_SET);
    adc_bemf_current_channel = adc_bemf_channel[2];
    break;
  case 4:
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_VAL);
    HAL_GPIO_WritePin(L6230_EN_CH2_GPIO_Port, L6230_EN_CH2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(L6230_EN_CH1_GPIO_Port, L6230_EN_CH1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH3_GPIO_Port, L6230_EN_CH3_Pin, GPIO_PIN_SET);
    adc_bemf_current_channel = adc_bemf_channel[1];
    break;
  case 5:
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_VAL);
    HAL_GPIO_WritePin(L6230_EN_CH1_GPIO_Port, L6230_EN_CH1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(L6230_EN_CH2_GPIO_Port, L6230_EN_CH2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L6230_EN_CH3_GPIO_Port, L6230_EN_CH3_Pin, GPIO_PIN_SET);
    adc_bemf_current_channel = adc_bemf_channel[0];
    break;
  }
}

void SixStep_SetCurrentLimit(unsigned int current_limit)
{

}

unsigned int SixStep_GetVBUS(void)
{
  unsigned long int vbus_v = ((((unsigned long int)3300 << 16) / 4096) * adc_vbus) >> 16;
  return vbus_v;

}

unsigned int SixStep_GetCurrent(void)
{
  unsigned long int curr_v = ((((unsigned int)3300 << 16) / 4096) * adc_current) >> 16;
  return curr_v;
}

unsigned int SixStep_GetTemp(void)
{
  unsigned long int temp_v = ((((unsigned long int)3300 << 16) / 4096) * adc_temp) >> 16;
  unsigned long int r2 = 4700;
  unsigned long int r1 = ((3300 * r2) / temp_v) - r2;
  return r1;
}

int SixStep_GetFilteredSpeed(void)
{
  return speed_filtered;
}
