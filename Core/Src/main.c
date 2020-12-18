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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Delay between ADC end of calibration and ADC enable.                     */
/* Delay estimation in CPU cycles: Case of ADC enable done                  */
/* immediately after ADC calibration, ADC clock setting slow                */
/* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
/* (CPU clock / ADC clock) is above 32.                                     */
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Variables for ADC conversion data */
__IO uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]; /* ADC group regular conversion data */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    while (!LL_USART_IsActiveFlag_TXE(DEBUGPORT));

    /* If last char to be sent, clear TC flag */
    if (ch == '\n') {
        LL_USART_ClearFlag_TC(DEBUGPORT);
    }

    /* Write character in Transmit Data register.
        TXE flag is cleared by writing data in TDR register */
    LL_USART_TransmitData8(DEBUGPORT, ch);


    /* Wait for TC flag to be raised for last char */
    while (!LL_USART_IsActiveFlag_TC(DEBUGPORT));

    return 0;
}
void Activate_ADC(void)
{
    __IO uint32_t wait_loop_index = 0U;
    __IO uint32_t backup_setting_adc_dma_transfer = 0U;

    /*## Operation on ADC hierarchical scope: ADC instance #####################*/

    /* Note: Hardware constraint (refer to description of the functions         */
    /*       below):                                                            */
    /*       On this STM32 series, setting of these features is conditioned to  */
    /*       ADC state:                                                         */
    /*       ADC must be disabled.                                              */
    /* Note: In this example, all these checks are not necessary but are        */
    /*       implemented anyway to show the best practice usages                */
    /*       corresponding to reference manual procedure.                       */
    /*       Software can be optimized by removing some of these checks, if     */
    /*       they are not relevant considering previous settings and actions    */
    /*       in user application.                                               */
    if (LL_ADC_IsEnabled(ADC1) == 0)
    {
        /* Enable ADC internal voltage regulator */
        LL_ADC_EnableInternalRegulator(ADC1);

        /* Delay for ADC internal voltage regulator stabilization.                */
        /* Compute number of CPU cycles to wait for, from delay in us.            */
        /* Note: Variable divided by 2 to compensate partially                    */
        /*       CPU processing cycles (depends on compilation optimization).     */
        /* Note: If system core clock frequency is below 200kHz, wait time        */
        /*       is only a few CPU processing cycles.                             */
        wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
        while(wait_loop_index != 0)
        {
            wait_loop_index--;
        }

        /* Disable ADC DMA transfer request during calibration */
        /* Note: Specificity of this STM32 series: Calibration factor is          */
        /*       available in data register and also transferred by DMA.          */
        /*       To not insert ADC calibration factor among ADC conversion data   */
        /*       in DMA destination address, DMA transfer must be disabled during */
        /*       calibration.                                                     */
        backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
        LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);

        /* Run ADC self calibration */
        LL_ADC_StartCalibration(ADC1);

        /* Poll for ADC effectively calibrated */
        while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);

        /* Restore ADC DMA transfer request after calibration */
        LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);

        /* Delay between ADC end of calibration and ADC enable.                   */
        /* Note: Variable divided by 2 to compensate partially                    */
        /*       CPU processing cycles (depends on compilation optimization).     */
        wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
        while(wait_loop_index != 0)
        {
            wait_loop_index--;
        }

        /* Enable ADC */
        LL_ADC_Enable(ADC1);

        /* Poll for ADC ready to convert */
        while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
        /* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
        /*       status afterwards.                                               */
        /*       This flag should be cleared at ADC Deactivation, before a new    */
        /*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
    }

    /*## Operation on ADC hierarchical scope: ADC group regular ################*/
    /* Note: No operation on ADC group regular performed here.                  */
    /*       ADC group regular conversions to be performed after this function  */
    /*       using function:                                                    */
    /*       "LL_ADC_REG_StartConversion();"                                    */

    /*## Operation on ADC hierarchical scope: ADC group injected ###############*/
    /* Note: Feature not available on this STM32 series */

}

/**
  * @brief  DMA transfer complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
void AdcDmaTransferComplete_Callback() {
    /* Computation of ADC conversions raw data to physical values               */
    /* using LL ADC driver helper macro.                                        */
    /* Management of the 2nd half of the buffer */
    //aADCxConvertedData_Voltage_mVolt[tmp_index] = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, aADCxConvertedData[tmp_index], LL_ADC_RESOLUTION_12B);
    printf("AdcDmaTransferComplete: %d\n", aADCxConvertedData[0]);
}

/**
  * @brief  DMA half transfer callback
  * @note   This function is executed when the half transfer interrupt
  *         is generated
  * @retval None
  */
void AdcDmaTransferHalf_Callback() {
    /* Computation of ADC conversions raw data to physical values               */
    /* using LL ADC driver helper macro.                                        */
    /* Management of the 1st half of the buffer */
//        aADCxConvertedData_Voltage_mVolt[tmp_index] = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, aADCxConvertedData[tmp_index], LL_ADC_RESOLUTION_12B);
    printf("AdcDmaTransferHalf: %d\n", aADCxConvertedData[0]);
}

/**
  * @brief  DMA transfer error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
void AdcDmaTransferError_Callback() {
    /* Error detected during DMA transfer */
    printf("AdcDmaTransferError\n");
}

/**
  * @brief  ADC group regular overrun interruption callback
  * @note   This function is executed when ADC group regular
  *         overrun error occurs.
  * @retval None
  */
void AdcGrpRegularOverrunError_Callback(void) {
    /* Note: Disable ADC interruption that caused this error before entering in */
    /*       infinite loop below.                                               */

    /* Disable ADC group regular overrun interruption */
    LL_ADC_DisableIT_OVR(ADC1);

    printf("AdcGrpRegularOverrunError\n");
}

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

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  LL_SYSCFG_DisableDBATT(LL_SYSCFG_UCPD1_STROBE | LL_SYSCFG_UCPD2_STROBE);

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

    /* Select ADC as DMA transfer request */
    LL_DMAMUX_SetRequestID(DMAMUX1,
                           LL_DMAMUX_CHANNEL_0,
                           LL_DMAMUX_REQ_ADC1);

    /* Set DMA transfer addresses of source and destination */
    LL_DMA_ConfigAddresses(DMA1,
                           LL_DMA_CHANNEL_1,
                           LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                           (uint32_t)&aADCxConvertedData,
                           LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    /* Set DMA transfer size */
    LL_DMA_SetDataLength(DMA1,
                         LL_DMA_CHANNEL_1,
                         ADC_CONVERTED_DATA_BUFFER_SIZE);

    /* Enable DMA transfer interruption: transfer complete */
    LL_DMA_EnableIT_TC(DMA1,
                       LL_DMA_CHANNEL_1);

    /* Enable DMA transfer interruption: half transfer */
    LL_DMA_EnableIT_HT(DMA1,
                       LL_DMA_CHANNEL_1);

    /* Enable DMA transfer interruption: transfer error */
    LL_DMA_EnableIT_TE(DMA1,
                       LL_DMA_CHANNEL_1);

    /*## Activation of DMA #####################################################*/
    /* Enable the DMA transfer */
    LL_DMA_EnableChannel(DMA1,
                         LL_DMA_CHANNEL_1);

    /* Configuration of ADC interruptions */
    /* Enable interruption ADC group regular overrun */
    LL_ADC_EnableIT_OVR(ADC1);

    /* Activate ADC */
    /* Perform ADC activation procedure to make it ready to convert. */
    Activate_ADC();

    /* Start ADC group regular conversion */
    /* Note: Hardware constraint (refer to description of the function          */
    /*       below):                                                            */
    /*       On this STM32 series, setting of this feature is conditioned to    */
    /*       ADC state:                                                         */
    /*       ADC must be enabled without conversion on going on group regular,  */
    /*       without ADC disable command on going.                              */
    /* Note: In this example, all these checks are not necessary but are        */
    /*       implemented anyway to show the best practice usages                */
    /*       corresponding to reference manual procedure.                       */
    /*       Software can be optimized by removing some of these checks, if     */
    /*       they are not relevant considering previous settings and actions    */
    /*       in user application.                                               */
    if ((LL_ADC_IsEnabled(ADC1) == 1) &&
        (LL_ADC_IsDisableOngoing(ADC1) == 0) &&
        (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)) {
        LL_ADC_REG_StartConversion(ADC1);
    } else {
        printf("Error: ADC conversion start could not be performed\n");
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  int count = 0;
  while (1)
  {
      LL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
      LL_mDelay(1000);
      printf("bump %d\n", count++);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
#pragma clang diagnostic pop
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_ADC(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 9, LL_RCC_PLLP_DIV_15);
  LL_RCC_PLL_EnableDomain_ADC();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 9, LL_RCC_PLLR_DIV_3);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(48000000);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PLL);
}

/* USER CODE BEGIN 4 */

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
