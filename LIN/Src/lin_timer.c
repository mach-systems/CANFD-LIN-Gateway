/*
 * lin_timer.c
 *
 *  Created on: Sep 18, 2020
 *      Author: Karel Hevessy
 */

#include <stm32g4xx_ll_tim.h>
#include "lin_timer.h"
#include "lin.h"

void LinTimeoutTimerInit(void)
{
  /* Get timer peripheral clock frequency */
  uint32_t freq = HAL_RCC_GetHCLKFreq();

  LL_TIM_InitTypeDef initStruct;
  initStruct.Prescaler = 0;
  initStruct.CounterMode = LL_TIM_COUNTERDIRECTION_UP;
  //initStruct.Autoreload = (freq / (uint32_t)(_LIN_SPEED_KBPS_ * 1000)) - 1;
  // test if working
  initStruct.Autoreload = (freq / (uint32_t)(_LIN_SPEED_BPS_)) - 1;
  initStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  initStruct.RepetitionCounter = 0;
  LL_TIM_Init(LIN_TIMEOUT_TIMER, &initStruct);
}

void LinTimeoutTimerStart(void)
{
  /* Reset the timer counter value */
  LL_TIM_SetCounter(LIN_TIMEOUT_TIMER, 0);
  /* Enable IRQ */
  LL_TIM_EnableIT_UPDATE(LIN_TIMEOUT_TIMER);
  /* Enable the timer */
  LL_TIM_EnableCounter(LIN_TIMEOUT_TIMER);
}
void LinTimeoutTimerStop(void)
{
  /* Enable IRQ */
  LL_TIM_DisableIT_UPDATE(LIN_TIMEOUT_TIMER);
  /* Disable the timer */
  LL_TIM_DisableCounter(LIN_TIMEOUT_TIMER);
}

void LinTimeoutTimerIrqCallback(void)
{
  LL_TIM_ClearFlag_UPDATE(LIN_TIMEOUT_TIMER);

  LinTimeoutTimerCallback();
}
