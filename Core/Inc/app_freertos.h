/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app_freertos.h
  * @brief          : Header for app_freertos.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_FREERTOS_H
#define __APP_FREERTOS_H

#ifdef __cplusplus
extern "C" {
#endif

extern osTimerId_t advLowPowerTimerHandle;
extern osTimerId_t HRSAPPMeasurementsTimerHandle;

extern osSemaphoreId_t advertisingSemaphoreHandle;

extern osMessageQueueId_t advertisingCmdQueueHandle;


/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
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
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __APP_FREERTOS_H */
