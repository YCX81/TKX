/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs.h"
#include "gpio.h"
#include "sdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  LED_MODE_OFF = 0,
  LED_MODE_SLOW_BLINK,
  LED_MODE_FAST_BLINK,
  LED_MODE_ON,
  LED_MODE_COUNT
} LED_Mode_t;

typedef enum { KEY_STATE_RELEASED = 0, KEY_STATE_PRESSED } Key_State_t;

typedef enum { SD_CARD_NOT_DETECTED = 0, SD_CARD_DETECTED } SD_Card_State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_DEBOUNCE_TIME 50        // 50ms debounce time
#define LED_SLOW_BLINK_PERIOD 1000  // 1000ms period for slow blink
#define LED_FAST_BLINK_PERIOD 200   // 200ms period for fast blink
#define SD_CARD_DEBOUNCE_TIME 100   // 100ms debounce time for SD card detection
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static LED_Mode_t current_led_mode = LED_MODE_OFF;
static Key_State_t last_key_state = KEY_STATE_RELEASED;
static SD_Card_State_t last_sd_card_state = SD_CARD_NOT_DETECTED;
static uint8_t sd_card_initialized = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for keyTask */
osThreadId_t keyTaskHandle;
const osThreadAttr_t keyTask_attributes = {
  .name = "keyTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sdCardTask */
osThreadId_t sdCardTaskHandle;
const osThreadAttr_t sdCardTask_attributes = {
  .name = "sdCardTask",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartKeyTask(void *argument);
void StartLedTask(void *argument);
void StartSDCardTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartKeyTask(void *argument);
void StartLedTask(void *argument);
void StartSDCardTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of keyTask */
  keyTaskHandle = osThreadNew(StartKeyTask, NULL, &keyTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);

  /* creation of sdCardTask */
  sdCardTaskHandle = osThreadNew(StartSDCardTask, NULL, &sdCardTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;
  /* Infinite loop */
  for (;;) {
    // Simple system monitoring - can be used for watchdog, statistics, etc.
    osDelay(1000); // Run every 1 second
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartKeyTask */
/**
* @brief Function implementing the keyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKeyTask */
void StartKeyTask(void *argument)
{
  /* USER CODE BEGIN StartKeyTask */
  (void)argument;
  Key_State_t current_key_state;
  
  for(;;)
  {
    // Read current key state (assuming active low)
    current_key_state = (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) ? 
                        KEY_STATE_PRESSED : KEY_STATE_RELEASED;
    
    // Check for key press (rising edge from released to pressed)
    if (current_key_state == KEY_STATE_PRESSED && last_key_state == KEY_STATE_RELEASED)
    {
      // Key pressed, switch to next mode
      current_led_mode = (current_led_mode + 1) % LED_MODE_COUNT;
      
      // Wait for debounce
      osDelay(KEY_DEBOUNCE_TIME);
    }
    
    last_key_state = current_key_state;
    osDelay(10); // Check key every 10ms
  }
  /* USER CODE END StartKeyTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  (void)argument;
  uint32_t led_counter = 0;
  GPIO_PinState led_state = GPIO_PIN_RESET;
  
  for(;;)
  {
    switch(current_led_mode)
    {
      case LED_MODE_OFF:
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        osDelay(100);
        break;
        
      case LED_MODE_SLOW_BLINK:
        led_counter++;
        if (led_counter >= (LED_SLOW_BLINK_PERIOD / 100))
        {
          led_state = (led_state == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
          HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, led_state);
          led_counter = 0;
        }
        osDelay(100);
        break;
        
      case LED_MODE_FAST_BLINK:
        led_counter++;
        if (led_counter >= (LED_FAST_BLINK_PERIOD / 100))
        {
          led_state = (led_state == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
          HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, led_state);
          led_counter = 0;
        }
        osDelay(100);
        break;
        
      case LED_MODE_ON:
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        osDelay(100);
        break;
        
      default:
        current_led_mode = LED_MODE_OFF;
        break;
    }
  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartSDCardTask */
/**
* @brief Function implementing the sdCardTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSDCardTask */
void StartSDCardTask(void *argument)
{
  /* USER CODE BEGIN StartSDCardTask */
  (void)argument;
  SD_Card_State_t current_sd_card_state;

  for (;;) {
    // Read current SD card detection state (assuming active low - card present
    // when pin is low)
    current_sd_card_state =
        (HAL_GPIO_ReadPin(SD_CARD_DET_GPIO_Port, SD_CARD_DET_Pin) ==
         GPIO_PIN_RESET)
            ? SD_CARD_DETECTED
            : SD_CARD_NOT_DETECTED;

    // Check for SD card insertion (change from not detected to detected)
    if (current_sd_card_state == SD_CARD_DETECTED &&
        last_sd_card_state == SD_CARD_NOT_DETECTED) {
      // SD card inserted, wait for debounce
      osDelay(SD_CARD_DEBOUNCE_TIME);

      // Re-check state after debounce
      current_sd_card_state =
          (HAL_GPIO_ReadPin(SD_CARD_DET_GPIO_Port, SD_CARD_DET_Pin) ==
           GPIO_PIN_RESET)
              ? SD_CARD_DETECTED
              : SD_CARD_NOT_DETECTED;

      if (current_sd_card_state == SD_CARD_DETECTED && !sd_card_initialized) {
        // Initialize SD card with simplified error handling
        HAL_StatusTypeDef sd_init_result = HAL_OK;

        // Reset SD handle before initialization
        HAL_SD_DeInit(&hsd);
        osDelay(50);  // Longer delay after deinit

        // Try to initialize SD card using enhanced function
        sd_init_result = SD_Init_Enhanced();

        // Check if initialization was successful
        if (sd_init_result == HAL_OK &&
            HAL_SD_GetState(&hsd) == HAL_SD_STATE_READY) {
          // Try to mount FATFS
          FRESULT mount_result = SD_Mount_FATFS();
          if (mount_result == FR_OK) {
            sd_card_initialized = 1;
            // SD card successfully initialized and mounted
          }
        } else {
          // SD card initialization failed, wait longer before next attempt
          osDelay(2000);  // Wait 2 seconds before next attempt
        }
      }
    }
    // Check for SD card removal (change from detected to not detected)
    else if (current_sd_card_state == SD_CARD_NOT_DETECTED &&
             last_sd_card_state == SD_CARD_DETECTED) {
      // SD card removed, wait for debounce
      osDelay(SD_CARD_DEBOUNCE_TIME);

      // Re-check state after debounce
      current_sd_card_state =
          (HAL_GPIO_ReadPin(SD_CARD_DET_GPIO_Port, SD_CARD_DET_Pin) ==
           GPIO_PIN_RESET)
              ? SD_CARD_DETECTED
              : SD_CARD_NOT_DETECTED;

      if (current_sd_card_state == SD_CARD_NOT_DETECTED &&
          sd_card_initialized) {
        // Deinitialize SD card
        HAL_SD_DeInit(&hsd);
        sd_card_initialized = 0;
        // You can add code here to unmount filesystem, etc.
      }
    }

    last_sd_card_state = current_sd_card_state;
    osDelay(50);  // Check SD card every 50ms
  }
  /* USER CODE END StartSDCardTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

