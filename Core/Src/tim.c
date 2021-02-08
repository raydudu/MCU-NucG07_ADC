/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

/* Set the pre-scaler value to have TIM1 counter clock equal to 10 kHz      */
/*
 In this example TIM1 input clock TIM1CLK is set to APB2 clock (PCLK2),
 since APB2 pre-scaler is equal to 1.
    TIM1CLK = PCLK2
    PCLK1 = HCLK
    => TIM1CLK = SystemCoreClock (56 MHz)
*/
#define PSC_10KHz __LL_TIM_CALC_PSC(SystemCoreClock, 10000)

/* Set the auto-reload value to have an initial update event frequency of 10 Hz */
/* TIM1CLK = SystemCoreClock / (APB prescaler & multiplier)                 */
#define TimOutClock (__LL_RCC_CALC_PCLK1_FREQ(SystemCoreClock, LL_RCC_GetAPB1Prescaler()) * \
    (LL_RCC_GetAPB1Prescaler() == LL_RCC_APB1_DIV_1 ? 1 : 2))

/* Set the auto-reload value to have an initial update event frequency of 10/1000 Hz */
/* TIM1CLK = SystemCoreClock / (APB prescaler & multiplier)                 */
#define ARR_10Hz __LL_TIM_CALC_ARR(TimOutClock, TIM_InitStruct.Prescaler, 10)
#define ARR_1KHz __LL_TIM_CALC_ARR(TimOutClock, TIM_InitStruct.Prescaler, 1000)



/* USER CODE END 0 */

/* TIM6 init function */
void MX_TIM6_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

  TIM_InitStruct.Prescaler = PSC_10KHz;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = ARR_1KHz;
  LL_TIM_Init(TIM6, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM6);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM6);

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
