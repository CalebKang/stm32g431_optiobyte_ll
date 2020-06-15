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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int32_t FLASH_WaitForLastOperation(uint32_t msTimeout)
{
	uint32_t loop = 0;
	uint32_t error;

  while(READ_BIT(FLASH->SR, FLASH_SR_BSY)) {
		if(loop++ > msTimeout){return -100;}
		LL_mDelay(1);
  }
	
	error = (FLASH->SR & FLASH_FLAG_SR_ERRORS);
  if (error != 0u){
		FLASH_CLEAR_FLAG(error);
		return -101;
	}
	
	if (FLASH_GET_FLAG(FLASH_FLAG_EOP)){
    /* Clear FLASH End of Operation pending bit */
    FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
  }
	
	return 0;
}

int32_t FLASH_Unlock(void)
{
  if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0U)
  {
    /* Authorize the FLASH Registers access */
    WRITE_REG(FLASH->KEYR, FLASH_KEY1);
    WRITE_REG(FLASH->KEYR, FLASH_KEY2);

    /* verify Flash is unlocked */
    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0U)
    {
      return -1;
    }
  }

  return 0;
}

int32_t FLASH_Lock(void)
{
  /* Set the LOCK Bit to lock the FLASH Registers access */
  SET_BIT(FLASH->CR, FLASH_CR_LOCK);

  /* verify Flash is locked */
  if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0U)
  {
		return 0;
  }

  return -1;
}

int32_t FLASH_OB_Unlock(void)
{
  if (READ_BIT(FLASH->CR, FLASH_CR_OPTLOCK) != 0U)
  {
    /* Authorizes the Option Byte register programming */
    WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY1);
    WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY2);

    /* verify option bytes are unlocked */
    if (READ_BIT(FLASH->CR, FLASH_CR_OPTLOCK) != 0U)
    {
			return -1;
    }
  }

  return 0;
}

int32_t FLASH_OB_Lock(void)
{
  /* Set the OPTLOCK Bit to lock the FLASH Option Byte Registers access */
  SET_BIT(FLASH->CR, FLASH_CR_OPTLOCK);

  /* Verify option bytes are locked */
  if (READ_BIT(FLASH->CR, FLASH_CR_OPTLOCK) != 0U)
  {
		return 0;
  }

  return -1;
}

int32_t FLASH_OB_Launch(void)
{
  /* Set the bit to force the option byte reloading */
  SET_BIT(FLASH->CR, FLASH_CR_OBL_LAUNCH);

  /* Wait for last operation to be completed */
  return FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_EnableBkUpAccess();
  LL_RCC_ForceBackupDomainReset();
  LL_RCC_ReleaseBackupDomainReset();
  LL_PWR_DisableBkUpAccess();
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_PWR);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /* Peripheral interrupt init*/
  /* PVD_PVM_IRQn interrupt configuration */
  NVIC_SetPriority(PVD_PVM_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(PVD_PVM_IRQn);

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral 
  */
  LL_PWR_DisableDeadBatteryPD();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
	
	/* PVD */
	/* VPVD0 around 2.0 V */
	/* VPVD1 around 2.2 V */
	/* VPVD2 around 2.4 V */
	/* VPVD3 around 2.5 V */
	/* VPVD4 around 2.6 V */
	/* VPVD5 around 2.8 V */
	/* VPVD6 around 2.9 V */
	/* External input analog voltage   (Compare internally to VREFINT) */
	LL_PWR_SetPVDLevel(LL_PWR_PVDLEVEL_6);
	LL_PWR_EnablePVD();
	
	/* Enable the PVD Extended Interrupt Line. */
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_16);
	
	/* Enable the PVD Event Line. */
	LL_EXTI_EnableEvent_0_31(LL_EXTI_LINE_16);
	
	/* Enable the PVD Extended Interrupt Rising Trigger. */
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_16);

  /* Enable the PVD Extended Interrupt Falling Trigger. */
	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_16);


	/* BOR Option byte */
	/* VBOR0 : 1.66 V */
	/* VBOR1 : 2.10 V */
	/* VBOR2 : 2.31 V */
	/* VBOR3 : 2.61 V */
	/* VBOR4 : 2.90 V */
	#define FLASH_CONFIG_BOR_LEVEL	(FLASH_OPTR_BOR_LEV_2)
	
  int32_t status = 0x00000000;

	if((READ_REG(FLASH->OPTR) & FLASH_OPTR_BOR_LEV) != FLASH_CONFIG_BOR_LEVEL)
	{
		LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8);

		/* Unlock Flash *************************************************/
		status |= FLASH_Unlock();
		
		/* Unlock the Options Bytes *************************************************/
		status |= FLASH_OB_Unlock();
		
		FLASH_CLEAR_FLAG(FLASH_FLAG_SR_ERRORS);
  
		status |= FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
		
		if (status == 0x00000000)
		{
			/* Configure the option bytes register */
			MODIFY_REG(FLASH->OPTR, FLASH_OPTR_BOR_LEV, FLASH_CONFIG_BOR_LEVEL);

			/* Set OPTSTRT Bit */
			SET_BIT(FLASH->CR, FLASH_CR_OPTSTRT);

			/* Wait for last operation to be completed */
			status |= FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);		

			FLASH_OB_Launch();
		}

		FLASH_OB_Lock();
		FLASH_Lock();
		
		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5);
		LL_mDelay(1000);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_8);
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 85, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  /* Insure 1?s transition state at intermediate medium speed clock based on DWT */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
  while(DWT->CYCCNT < 100);
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);

  LL_Init1msTick(170000000);

  LL_SetSystemCoreClock(170000000);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
