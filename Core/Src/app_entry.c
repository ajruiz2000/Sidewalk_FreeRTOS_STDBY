/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_entry.c
  * @author  MCD Application Team
  * @brief   Entry point of the application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_common.h"
#include "app_conf.h"
#include "main.h"
#include "app_entry.h"
#include "cmsis_os2.h"
#include "app_freertos.h"
#include "task.h"
#if (CFG_LPM_LEVEL != 0)
#include "stm32_lpm.h"
#endif /* (CFG_LPM_LEVEL != 0) */
#include "stm32_timer.h"
#include "stm32_mm.h"
#if (CFG_LOG_SUPPORTED != 0)
#include "stm32_adv_trace.h"
#include "serial_cmd_interpreter.h"
#endif /* CFG_LOG_SUPPORTED */
#include "app_ble.h"
#include "ll_sys_if.h"
#include "app_sys.h"
#include "otp.h"
#include "scm.h"
#include "bpka.h"
#include "ll_sys.h"
#include "advanced_memory_manager.h"
#include "flash_driver.h"
#include "flash_manager.h"
#include "simple_nvm_arbiter.h"
#include "app_debug.h"
#if (USE_TEMPERATURE_BASED_RADIO_CALIBRATION == 1)
#include "adc_ctrl.h"
#endif /* USE_TEMPERATURE_BASED_RADIO_CALIBRATION */
#include "timer_if.h"
extern void xPortSysTickHandler (void);
extern void vPortSetupTimerInterrupt(void);
/* Private includes -----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
/* AMM_BCKGND_TASK related defines */
#define AMM_TASK_STACK_SIZE                     (128 * 4)
#define AMM_TASK_PRIO                           (osPriorityNormal)

/* BPKA_TASK related defines */
#define BPKA_TASK_STACK_SIZE                    (128 * 4)
#define BPKA_TASK_PRIO                          (osPriorityNormal)

/* HW_RNG_TASK related defines */
#define HW_RNG_TASK_STACK_SIZE                  (128 * 4)
#define HW_RNG_TASK_PRIO                        (osPriorityNormal)

/* FLASH_MANAGER_TASK related defines */
#define FLASH_MANAGER_TASK_STACK_SIZE           (256 * 4)
#define FLASH_MANAGER_TASK_PRIO                 (osPriorityNormal)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private constants ---------------------------------------------------------*/
/* USER CODE BEGIN PC */

/* USER CODE END PC */

/* Private variables ---------------------------------------------------------*/
#if ( CFG_LPM_LEVEL != 0)
static bool system_startup_done = FALSE;
/* Holds maximum number of FreeRTOS tick periods that can be suppressed */
static uint32_t maximumPossibleSuppressedTicks = 0;
#endif /* ( CFG_LPM_LEVEL != 0) */

#if (CFG_LOG_SUPPORTED != 0)
/* Log configuration */
static Log_Module_t Log_Module_Config = { .verbose_level = APPLI_CONFIG_LOG_LEVEL, .region = LOG_REGION_ALL_REGIONS };
#endif /* (CFG_LOG_SUPPORTED != 0) */

static uint32_t AMM_Pool[CFG_AMM_POOL_SIZE];
static AMM_VirtualMemoryConfig_t vmConfig[CFG_AMM_VIRTUAL_MEMORY_NUMBER] =
{
  /* Virtual Memory #1 */
  {
    .Id = CFG_AMM_VIRTUAL_STACK_BLE,
    .BufferSize = CFG_AMM_VIRTUAL_STACK_BLE_BUFFER_SIZE
  },
  /* Virtual Memory #2 */
  {
    .Id = CFG_AMM_VIRTUAL_APP_BLE,
    .BufferSize = CFG_AMM_VIRTUAL_APP_BLE_BUFFER_SIZE
  },
};

static AMM_InitParameters_t ammInitConfig =
{
  .p_PoolAddr = AMM_Pool,
  .PoolSize = CFG_AMM_POOL_SIZE,
  .VirtualMemoryNumber = CFG_AMM_VIRTUAL_MEMORY_NUMBER,
  .p_VirtualMemoryConfigList = vmConfig
};

osSemaphoreId_t       HwRngSemaphore;
osThreadId_t          HwRngThread;

osSemaphoreId_t       AmmSemaphore;
osThreadId_t          AmmThread;

osSemaphoreId_t       BpkaSemaphore;
osThreadId_t          BpkaThread;

osSemaphoreId_t       FlashManagerSemaphore;
osThreadId_t          FlashManagerThread;

static UTIL_TIMER_Object_t TimerOStick_Id;
static UTIL_TIMER_Object_t TimerOSwakeup_Id;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Global variables ----------------------------------------------------------*/
/* USER CODE BEGIN GV */

/* USER CODE END GV */

/* Private functions prototypes-----------------------------------------------*/
static void Config_HSE(void);
static void RNG_Init( void );
static void System_Init( void );
static void SystemPower_Config( void );
static void TimerOStickCB(void *arg);
static void TimerOSwakeupCB(void *arg);
#if ( CFG_LPM_LEVEL != 0)
static void preOSsleepProcessing(uint32_t expectedIdleTime);
static void postOSsleepProcessing(uint32_t expectedIdleTime);
static uint32_t getCurrentTime(void);
#endif /* CFG_LPM_LEVEL */

/**
 * @brief Wrapper for init function of the MM for the AMM
 *
 * @param p_PoolAddr: Address of the pool to use - Not use -
 * @param PoolSize: Size of the pool - Not use -
 *
 * @return None
 */
static void AMM_WrapperInit (uint32_t * const p_PoolAddr, const uint32_t PoolSize);

/**
 * @brief Wrapper for allocate function of the MM for the AMM
 *
 * @param BufferSize
 *
 * @return Allocated buffer
 */
static uint32_t * AMM_WrapperAllocate (const uint32_t BufferSize);

/**
 * @brief Wrapper for free function of the MM for the AMM
 *
 * @param p_BufferAddr
 *
 * @return None
 */
static void AMM_WrapperFree (uint32_t * const p_BufferAddr);

static void BPKA_Task_Entry(void* thread_input);
static void AMM_Task_Entry(void* thread_input);
static void FM_Task_Entry(void* thread_input);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Functions Definition ------------------------------------------------------*/
/**
 * @brief   System Initialisation.
 */
void MX_APPE_Config(void)
{
  /* Configure HSE Tuning */
  Config_HSE();
}

/**
 * @brief   System Initialisation.
 */
uint32_t MX_APPE_Init(void *p_param)
{
  APP_DEBUG_SIGNAL_SET(APP_APPE_INIT);

  UNUSED(p_param);

  /* System initialization */
  System_Init();

  /* Configure the system Power Mode */
  SystemPower_Config();

  /* Initialize the Advance Memory Manager */
  AMM_Init (&ammInitConfig);

  /* Register the AMM background task */
  const osSemaphoreAttr_t AmmSemaphore_attributes = {
    .name = "AMM Semaphore"
  };
  AmmSemaphore = osSemaphoreNew(1U, 0U, &AmmSemaphore_attributes);
  if (AmmSemaphore == NULL)
  {
    Error_Handler();
  }

  const osThreadAttr_t AmmTask_attributes = {
    .name = "AMM Task",
    .priority = (osPriority_t)AMM_TASK_PRIO,
    .stack_size = AMM_TASK_STACK_SIZE
  };
  AmmThread = osThreadNew(AMM_Task_Entry, NULL, &AmmTask_attributes);
  if (AmmThread == NULL)
  {
    Error_Handler();
  }
  /* Initialize the Simple NVM Arbiter */
  SNVMA_Init ((uint32_t *)CFG_SNVMA_START_ADDRESS);

  /* Register the flash manager task */
  const osSemaphoreAttr_t FlashManagerSemaphore_attributes = {
    .name = "Flash Manager Semaphore"
  };
  FlashManagerSemaphore = osSemaphoreNew(1U, 0U, &FlashManagerSemaphore_attributes);
  if (FlashManagerSemaphore == NULL)
  {
    Error_Handler();
  }

  const osThreadAttr_t FlashManagerTask_attributes = {
    .name = "Flash Manager Task",
    .priority = (osPriority_t)FLASH_MANAGER_TASK_PRIO,
    .stack_size = FLASH_MANAGER_TASK_STACK_SIZE
  };
  FlashManagerThread = osThreadNew(FM_Task_Entry, NULL, &FlashManagerTask_attributes);
  if (FlashManagerThread == NULL)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN APPE_Init_1 */

  /* USER CODE END APPE_Init_1 */
  const osSemaphoreAttr_t BpkaSemaphore_attributes = {
    .name = "BPKA Semaphore"
  };
  BpkaSemaphore = osSemaphoreNew(1U, 0U, &BpkaSemaphore_attributes);
  if (BpkaSemaphore == NULL)
  {
    Error_Handler();
  }

  const osThreadAttr_t BpkaTask_attributes = {
    .name = "BPKA Task",
    .priority = (osPriority_t)BPKA_TASK_PRIO,
    .stack_size = BPKA_TASK_STACK_SIZE
  };
  BpkaThread = osThreadNew(BPKA_Task_Entry, NULL, &BpkaTask_attributes);
  if (BpkaThread == NULL)
  {
    Error_Handler();
  }

  BPKA_Reset( );

  RNG_Init();

  /* Disable flash before any use - RFTS */
  FD_SetStatus (FD_FLASHACCESS_RFTS, LL_FLASH_DISABLE);
  /* Enable RFTS Bypass for flash operation - Since LL has not started yet */
  FD_SetStatus (FD_FLASHACCESS_RFTS_BYPASS, LL_FLASH_ENABLE);
  /* Enable flash system flag */
  FD_SetStatus (FD_FLASHACCESS_SYSTEM, LL_FLASH_ENABLE);

  APP_BLE_Init();

  /* create a SW timer to wakeup system from low power */
  UTIL_TIMER_Create(&TimerOSwakeup_Id,
                    0,
                    UTIL_TIMER_ONESHOT,
                    &TimerOSwakeupCB, 0);
#if ( CFG_LPM_LEVEL != 0)
  maximumPossibleSuppressedTicks = UINT32_MAX;// TODO check this value
#endif /* ( CFG_LPM_LEVEL != 0) */

  /* Disable RFTS Bypass for flash operation - Since LL has not started yet */
  FD_SetStatus (FD_FLASHACCESS_RFTS_BYPASS, LL_FLASH_DISABLE);

  /* USER CODE BEGIN APPE_Init_2 */

  /* USER CODE END APPE_Init_2 */
  APP_DEBUG_SIGNAL_RESET(APP_APPE_INIT);
  return WPAN_SUCCESS;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/**
 * @brief Configure HSE by read this Tuning from OTP
 *
 */
static void Config_HSE(void)
{
  OTP_Data_s* otp_ptr = NULL;

  /* Read HSE_Tuning from OTP */
  if (OTP_Read(DEFAULT_OTP_IDX, &otp_ptr) != HAL_OK)
  {
    /* OTP no present in flash, apply default gain */
    HAL_RCCEx_HSESetTrimming(0x0C);
  }
  else
  {
    HAL_RCCEx_HSESetTrimming(otp_ptr->hsetune);
  }
}

/**
 *
 */
static void System_Init( void )
{
  /* Clear RCC RESET flag */
  LL_RCC_ClearResetFlags();

  UTIL_TIMER_Init();

  /* Enable wakeup out of standby from RTC ( UTIL_TIMER )*/
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN7_HIGH_3);

#if (CFG_LOG_SUPPORTED != 0)
  /* Initialize the logs ( using the USART ) */
  Log_Module_Init( Log_Module_Config );

  /* Initialize the Command Interpreter */
  Serial_CMD_Interpreter_Init();
#endif  /* (CFG_LOG_SUPPORTED != 0) */

#if (USE_TEMPERATURE_BASED_RADIO_CALIBRATION == 1)
  adc_ctrl_init();
#endif /* USE_TEMPERATURE_BASED_RADIO_CALIBRATION */

#if ( CFG_LPM_LEVEL != 0)
  system_startup_done = TRUE;
#endif /* ( CFG_LPM_LEVEL != 0) */

  return;
}

/**
 * @brief  Configure the system for power optimization
 *
 * @note  This API configures the system to be ready for low power mode
 *
 * @param  None
 * @retval None
 */
static void SystemPower_Config(void)
{
#if (CFG_SCM_SUPPORTED == 1)
  /* Initialize System Clock Manager */
  scm_init();
#endif /* CFG_SCM_SUPPORTED */

#if (CFG_DEBUGGER_LEVEL == 0)
  /* Pins used by SerialWire Debug are now analog input */
  GPIO_InitTypeDef DbgIOsInit = {0};
  DbgIOsInit.Mode = GPIO_MODE_ANALOG;
  DbgIOsInit.Pull = GPIO_NOPULL;
  DbgIOsInit.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &DbgIOsInit);

  DbgIOsInit.Mode = GPIO_MODE_ANALOG;
  DbgIOsInit.Pull = GPIO_NOPULL;
  DbgIOsInit.Pin = GPIO_PIN_3|GPIO_PIN_4;
  HAL_GPIO_Init(GPIOB, &DbgIOsInit);
#endif /* CFG_DEBUGGER_LEVEL */

#if (CFG_LPM_LEVEL != 0)
  /* Initialize low Power Manager. By default enabled */
  UTIL_LPM_Init();

#if (CFG_LPM_STDBY_SUPPORTED == 1)
  /* Enable SRAM1, SRAM2 and RADIO retention*/
  LL_PWR_SetSRAM1SBRetention(LL_PWR_SRAM1_SB_FULL_RETENTION);
  LL_PWR_SetSRAM2SBRetention(LL_PWR_SRAM2_SB_FULL_RETENTION);
  LL_PWR_SetRadioSBRetention(LL_PWR_RADIO_SB_FULL_RETENTION); /* Retain sleep timer configuration */

#else /* (CFG_LPM_STDBY_SUPPORTED == 1) */
  UTIL_LPM_SetOffMode(1U << CFG_LPM_APP, UTIL_LPM_DISABLE);
#endif /* (CFG_LPM_STDBY_SUPPORTED == 1) */
#endif /* (CFG_LPM_LEVEL != 0)  */
}

static void HW_RNG_Process_Task(void *argument)
{
  for(;;)
  {
    osSemaphoreAcquire(HwRngSemaphore, osWaitForever);
    osMutexAcquire(LinkLayerMutex, osWaitForever);
    HW_RNG_Process();
    osMutexRelease(LinkLayerMutex);
    osThreadYield();
  }
}

/**
 * @brief Initialize Random Number Generator module
 */
static void RNG_Init(void)
{
  HW_RNG_Start();

  /* Register Semaphore to launch the Random Process */
  HwRngSemaphore = osSemaphoreNew(1U, 0U, NULL );
  if ( HwRngSemaphore == NULL )
  {
    APP_DBG( "ERROR FREERTOS : RANDOM PROCESS SEMAPHORE CREATION FAILED" );
    while(1);
  }

  /* Create the Random Process Thread */
  const osThreadAttr_t HwRngTask_attributes = {
    .name = "RNG Task",
    .priority = (osPriority_t) HW_RNG_TASK_PRIO,
    .stack_size = HW_RNG_TASK_STACK_SIZE
  };
  HwRngThread = osThreadNew( HW_RNG_Process_Task, NULL, &HwRngTask_attributes );
  if ( HwRngThread == NULL )
  {
    APP_DBG( "ERROR FREERTOS : RANDOM PROCESS THREAD CREATION FAILED" );
    while(1);
  }
}

static void AMM_WrapperInit (uint32_t * const p_PoolAddr, const uint32_t PoolSize)
{
  UTIL_MM_Init ((uint8_t *)p_PoolAddr, ((size_t)PoolSize * sizeof(uint32_t)));
}

static uint32_t * AMM_WrapperAllocate (const uint32_t BufferSize)
{
  return (uint32_t *)UTIL_MM_GetBuffer (((size_t)BufferSize * sizeof(uint32_t)));
}

static void AMM_WrapperFree (uint32_t * const p_BufferAddr)
{
  UTIL_MM_ReleaseBuffer ((void *)p_BufferAddr);
}

/* OS tick callback */
static void TimerOStickCB(void *arg)
{
  xPortSysTickHandler();
  /* USER CODE BEGIN TimerOStickCB */
  HAL_IncTick();
  /* USER CODE END TimerOStickCB */

  return;
}

/* OS wakeup callback */
static void TimerOSwakeupCB(void *arg)
{
  /* USER CODE BEGIN TimerOSwakeupCB */

  /* USER CODE END TimerOSwakeupCB */
  return;
}

#if ( CFG_LPM_LEVEL != 0)
static void preOSsleepProcessing(uint32_t expectedIdleTime)
{
  LL_PWR_ClearFlag_STOP();

  if ( ( system_startup_done != FALSE ) && ( UTIL_LPM_GetMode() == UTIL_LPM_OFFMODE ) )
  {
    APP_SYS_BLE_EnterDeepSleep();
  }

  LL_RCC_ClearResetFlags();

#if defined(STM32WBAXX_SI_CUT1_0)
  /* Wait until HSE is ready */
  while (LL_RCC_HSE_IsReady() == 0);

  UTILS_ENTER_LIMITED_CRITICAL_SECTION(RCC_INTR_PRIO << 4U);
  scm_hserdy_isr();
  UTILS_EXIT_LIMITED_CRITICAL_SECTION();
#endif /* STM32WBAXX_SI_CUT1_0 */

  UTIL_LPM_EnterLowPower(); /* WFI instruction call is inside this API */
}

static void postOSsleepProcessing(uint32_t expectedIdleTime)
{
  UTIL_TIMER_Stop(&TimerOSwakeup_Id);

  LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_RADIO);
  ll_sys_dp_slp_exit();
}

/* return current time since boot, continue to count in standby low power mode */
static uint32_t getCurrentTime(void)
{
  return TIMER_IF_GetTimerValue();
}
#endif /* ( CFG_LPM_LEVEL != 0) */

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */

/* USER CODE END FD_LOCAL_FUNCTIONS */

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void BPKACB_Process( void )
{
  osSemaphoreRelease(BpkaSemaphore);
}

static void BPKA_Task_Entry(void *argument)
{
  for(;;)
  {
    osSemaphoreAcquire(BpkaSemaphore, osWaitForever);
    osMutexAcquire(LinkLayerMutex, osWaitForever);
    BPKA_BG_Process();
    osMutexRelease(LinkLayerMutex);
    osThreadYield();
  }
}

/**
 * @brief Callback used by 'Random Generator' to launch Task to generate Random Numbers
 */
void HWCB_RNG_Process( void )
{
  osSemaphoreRelease(HwRngSemaphore);
}

void AMM_RegisterBasicMemoryManager (AMM_BasicMemoryManagerFunctions_t * const p_BasicMemoryManagerFunctions)
{
  /* Fulfill the function handle */
  p_BasicMemoryManagerFunctions->Init = AMM_WrapperInit;
  p_BasicMemoryManagerFunctions->Allocate = AMM_WrapperAllocate;
  p_BasicMemoryManagerFunctions->Free = AMM_WrapperFree;
}

void AMM_ProcessRequest (void)
{
  /* Ask for AMM background task scheduling */
  osSemaphoreRelease(AmmSemaphore);
}

static void AMM_Task_Entry(void* argument)
{
  UNUSED(argument);

  while(1)
  {
    osSemaphoreAcquire(AmmSemaphore, osWaitForever);
    osMutexAcquire(LinkLayerMutex, osWaitForever);
    AMM_BackgroundProcess();
    osMutexRelease(LinkLayerMutex);
    osThreadYield();
  }
}

void FM_ProcessRequest (void)
{
  /* Schedule the background process */
  osSemaphoreRelease(FlashManagerSemaphore);
}

static void FM_Task_Entry(void* argument)
{
  UNUSED(argument);

  while(1)
  {
    osSemaphoreAcquire(FlashManagerSemaphore, osWaitForever);
    osMutexAcquire(LinkLayerMutex, osWaitForever);
    FM_BackgroundProcess();
    osMutexRelease(LinkLayerMutex);
    osThreadYield();
  }
}

#if (CFG_LOG_SUPPORTED != 0)
/**
 *
 */
void RNG_KERNEL_CLK_OFF(void)
{
  /* RNG module may not switch off HSI clock when traces are used */

  /* USER CODE BEGIN RNG_KERNEL_CLK_OFF */

  /* USER CODE END RNG_KERNEL_CLK_OFF */
}

void SCM_HSI_CLK_OFF(void)
{
  /* SCM module may not switch off HSI clock when traces are used */

  /* USER CODE BEGIN SCM_HSI_CLK_OFF */

  /* USER CODE END SCM_HSI_CLK_OFF */
}

void UTIL_ADV_TRACE_PreSendHook(void)
{
#if (CFG_LPM_LEVEL != 0)
  /* Disable Stop mode before sending a LOG message over UART */
  UTIL_LPM_SetStopMode(1U << CFG_LPM_LOG, UTIL_LPM_DISABLE);
#endif /* (CFG_LPM_LEVEL != 0) */
  /* USER CODE BEGIN UTIL_ADV_TRACE_PreSendHook */

  /* USER CODE END UTIL_ADV_TRACE_PreSendHook */
}

void UTIL_ADV_TRACE_PostSendHook(void)
{
#if (CFG_LPM_LEVEL != 0)
  /* Enable Stop mode after LOG message over UART sent */
  UTIL_LPM_SetStopMode(1U << CFG_LPM_LOG, UTIL_LPM_ENABLE);
#endif /* (CFG_LPM_LEVEL != 0) */
  /* USER CODE BEGIN UTIL_ADV_TRACE_PostSendHook */

  /* USER CODE END UTIL_ADV_TRACE_PostSendHook */
}

#endif /* (CFG_LOG_SUPPORTED != 0) */

/* Implement weak function to setup a timer that will trig OS ticks */
void vPortSetupTimerInterrupt( void )
{
  UTIL_TIMER_Create(&TimerOStick_Id,
                    portTICK_PERIOD_MS,
                    UTIL_TIMER_PERIODIC,
                    &TimerOStickCB, 0);

  UTIL_TIMER_StartWithPeriod(&TimerOStick_Id, portTICK_PERIOD_MS);
  /* USER CODE BEGIN vPortSetupTimerInterrupt */
  if(portTICK_PERIOD_MS == 10U)
  {
    uwTickFreq = HAL_TICK_FREQ_10HZ;
  }
  else if(portTICK_PERIOD_MS == 100U)
  {
    uwTickFreq = HAL_TICK_FREQ_100HZ;
  }
  else if(portTICK_PERIOD_MS == 1000U)
  {
    uwTickFreq = HAL_TICK_FREQ_1KHZ;
  }
  else
  {
    uwTickFreq = HAL_TICK_FREQ_DEFAULT;
  }
  /* USER CODE END vPortSetupTimerInterrupt */
  return;
}
#if ( CFG_LPM_LEVEL != 0)
void vPortSuppressTicksAndSleep( uint32_t xExpectedIdleTime )
{
  uint32_t lowPowerTimeBeforeSleep, lowPowerTimeAfterSleep;
  eSleepModeStatus eSleepStatus;

  /* Stop the timer that is generating the OS tick interrupt. */
  UTIL_TIMER_Stop(&TimerOStick_Id);

  /* Make sure the SysTick reload value does not overflow the counter. */
  if( xExpectedIdleTime > maximumPossibleSuppressedTicks )
  {
    xExpectedIdleTime = maximumPossibleSuppressedTicks;
  }

  /* Enter a critical section but don't use the taskENTER_CRITICAL()
   * method as that will mask interrupts that should exit sleep mode. */
  __asm volatile ( "cpsid i" ::: "memory" );
  __asm volatile ( "dsb" );
  __asm volatile ( "isb" );

  eSleepStatus = eTaskConfirmSleepModeStatus();

  /* If a context switch is pending or a task is waiting for the scheduler
   * to be unsuspended then abandon the low power entry. */
  if( eSleepStatus == eAbortSleep )
  {
    /* Restart the timer that is generating the OS tick interrupt. */
    UTIL_TIMER_StartWithPeriod(&TimerOStick_Id, portTICK_PERIOD_MS);

    /* Re-enable interrupts - see comments above the cpsid instruction above. */
    __asm volatile ( "cpsie i" ::: "memory" );
  }
  else
  {
    if( eSleepStatus == eNoTasksWaitingTimeout )
    {
      /* It is not necessary to configure an interrupt to bring the
         microcontroller out of its low power state at a fixed time in the
         future. */
      preOSsleepProcessing(xExpectedIdleTime); /* WFI instruction call is inside this API */
      postOSsleepProcessing(xExpectedIdleTime);
    }
    else
    {
      /* Configure an interrupt to bring the microcontroller out of its low
         power state at the time the kernel next needs to execute. The
         interrupt must be generated from a source that remains operational
         when the microcontroller is in a low power state. */
      UTIL_TIMER_StartWithPeriod(&TimerOSwakeup_Id, (xExpectedIdleTime - 1) * portTICK_PERIOD_MS);

      /* Read the current time from RTC, maintainned in standby */
      lowPowerTimeBeforeSleep = getCurrentTime();

      /* Enter the low power state. */
      preOSsleepProcessing(xExpectedIdleTime); /* WFI instruction call is inside this API */
      postOSsleepProcessing(xExpectedIdleTime);

      /* Determine how long the microcontroller was actually in a low power
         state for, which will be less than xExpectedIdleTime if the
         microcontroller was brought out of low power mode by an interrupt
         other than that configured by the vSetWakeTimeInterrupt() call.
         Note that the scheduler is suspended before
         portSUPPRESS_TICKS_AND_SLEEP() is called, and resumed when
         portSUPPRESS_TICKS_AND_SLEEP() returns.  Therefore no other tasks will
         execute until this function completes. */
      lowPowerTimeAfterSleep = getCurrentTime();

      /* Correct the kernel tick count to account for the time spent in its low power state. */
      vTaskStepTick( TIMER_IF_Convert_Tick2ms(lowPowerTimeAfterSleep - lowPowerTimeBeforeSleep) / portTICK_PERIOD_MS );

    }
    /* Re-enable interrupts to allow the interrupt that brought the MCU
     * out of sleep mode to execute immediately.  See comments above
     * the cpsid instruction above. */
    __asm volatile ( "cpsie i" ::: "memory" );
    __asm volatile ( "dsb" );
    __asm volatile ( "isb" );

    /* Restart the timer that is generating the OS tick interrupt. */
    UTIL_TIMER_StartWithPeriod(&TimerOStick_Id, portTICK_PERIOD_MS);
  }
  return;
}
#else
void vPortSuppressTicksAndSleep( uint32_t xExpectedIdleTime )
{
  return;
}
#endif /* ( CFG_LPM_LEVEL != 0) */
/* USER CODE BEGIN FD_WRAP_FUNCTIONS */

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  return HAL_OK;
}
/* USER CODE END FD_WRAP_FUNCTIONS */
