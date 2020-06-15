/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx.h"
#include "stm32g4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */
//Scott - copied from HAL driver and don't change
#define FLASH_KEY1                0x45670123U              /*!< Flash key1 */
#define FLASH_KEY2                0xCDEF89ABU              /*!< Flash key2: used with FLASH_KEY1 
                                                                to unlock the FLASH registers access */

#define FLASH_PDKEY1              0x04152637U              /*!< Flash power down key1 */
#define FLASH_PDKEY2              0xFAFBFCFDU              /*!< Flash power down key2: used with FLASH_PDKEY1 
                                                                to unlock the RUN_PD bit in FLASH_ACR */

#define FLASH_OPTKEY1             0x08192A3BU              /*!< Flash option byte key1 */
#define FLASH_OPTKEY2             0x4C5D6E7FU              /*!< Flash option byte key2: used with FLASH_OPTKEY1 to allow option bytes operations */

#define FLASH_FLAG_EOP            FLASH_SR_EOP             /*!< FLASH End of operation flag */
#define FLASH_FLAG_OPERR          FLASH_SR_OPERR           /*!< FLASH Operation error flag */
#define FLASH_FLAG_PROGERR        FLASH_SR_PROGERR         /*!< FLASH Programming error flag */
#define FLASH_FLAG_WRPERR         FLASH_SR_WRPERR          /*!< FLASH Write protection error flag */
#define FLASH_FLAG_PGAERR         FLASH_SR_PGAERR          /*!< FLASH Programming alignment error flag */
#define FLASH_FLAG_SIZERR         FLASH_SR_SIZERR          /*!< FLASH Size error flag  */
#define FLASH_FLAG_PGSERR         FLASH_SR_PGSERR          /*!< FLASH Programming sequence error flag */
#define FLASH_FLAG_MISERR         FLASH_SR_MISERR          /*!< FLASH Fast programming data miss error flag */
#define FLASH_FLAG_FASTERR        FLASH_SR_FASTERR         /*!< FLASH Fast programming error flag */
#define FLASH_FLAG_RDERR          FLASH_SR_RDERR           /*!< FLASH PCROP read error flag */
#define FLASH_FLAG_OPTVERR        FLASH_SR_OPTVERR         /*!< FLASH Option validity error flag  */
#define FLASH_FLAG_BSY            FLASH_SR_BSY             /*!< FLASH Busy flag */
#define FLASH_FLAG_ECCC           FLASH_ECCR_ECCC          /*!< FLASH ECC correction in 64 LSB bits */
#define FLASH_FLAG_ECCD           FLASH_ECCR_ECCD          /*!< FLASH ECC detection in 64 LSB bits */

#define FLASH_FLAG_SR_ERRORS      (FLASH_FLAG_OPERR   | FLASH_FLAG_PROGERR | FLASH_FLAG_WRPERR | \
                                   FLASH_FLAG_PGAERR  | FLASH_FLAG_SIZERR  | FLASH_FLAG_PGSERR | \
                                   FLASH_FLAG_MISERR  | FLASH_FLAG_FASTERR | FLASH_FLAG_RDERR  | \
                                   FLASH_FLAG_OPTVERR)
#define FLASH_FLAG_ECCR_ERRORS    (FLASH_FLAG_ECCC    | FLASH_FLAG_ECCD)
#define FLASH_FLAG_ALL_ERRORS     (FLASH_FLAG_SR_ERRORS | FLASH_FLAG_ECCR_ERRORS)

#define FLASH_CLEAR_FLAG(__FLAG__)        do { if(((__FLAG__) & FLASH_FLAG_ECCR_ERRORS) != 0U) { SET_BIT(FLASH->ECCR, ((__FLAG__) & FLASH_FLAG_ECCR_ERRORS)); }\
                                                     if(((__FLAG__) & ~(FLASH_FLAG_ECCR_ERRORS)) != 0U) { WRITE_REG(FLASH->SR, ((__FLAG__) & ~(FLASH_FLAG_ECCR_ERRORS))); }\
                                                   } while (0)
#define FLASH_GET_FLAG(__FLAG__)          ((((__FLAG__) & FLASH_FLAG_ECCR_ERRORS) != 0U) ? \
                                                 (READ_BIT(FLASH->ECCR, (__FLAG__)) == (__FLAG__)) : \
                                                 (READ_BIT(FLASH->SR,   (__FLAG__)) == (__FLAG__)))
#define FLASH_INSTRUCTION_CACHE_ENABLE()  SET_BIT(FLASH->ACR, FLASH_ACR_ICEN)
#define FLASH_INSTRUCTION_CACHE_DISABLE() CLEAR_BIT(FLASH->ACR, FLASH_ACR_ICEN)
#define FLASH_DATA_CACHE_ENABLE()         SET_BIT(FLASH->ACR, FLASH_ACR_DCEN)
#define FLASH_DATA_CACHE_DISABLE()        CLEAR_BIT(FLASH->ACR, FLASH_ACR_DCEN)
																									 
#define FLASH_INSTRUCTION_CACHE_RESET()   do { SET_BIT(FLASH->ACR, FLASH_ACR_ICRST);   \
                                                     CLEAR_BIT(FLASH->ACR, FLASH_ACR_ICRST); \
                                                   } while (0)
#define FLASH_DATA_CACHE_RESET()          do { SET_BIT(FLASH->ACR, FLASH_ACR_DCRST);   \
                                                     CLEAR_BIT(FLASH->ACR, FLASH_ACR_DCRST); \
                                                   } while (0)

#define FLASH_TIMEOUT_VALUE             1000U   /* 1 s  */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
