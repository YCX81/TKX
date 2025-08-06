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
#include "fps.h"
#include "gpio.h"
#include "lcd.h"
#include "lcd_init.h"
#include "sdio.h"
#include "stdbool.h"
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

typedef enum {
  LCD_MODE_OFF = 0,
  LCD_MODE_COLOR_BLOCKS,
  LCD_MODE_GRADIENT_STRIPES,
  LCD_MODE_CONCENTRIC_SQUARES,
  LCD_MODE_COLOR_GRID,
  LCD_MODE_RIPPLE_CIRCLES,
  LCD_MODE_TEXT_DISPLAY,
  LCD_MODE_COUNT
} LCD_Mode_t;

typedef enum {
  MSG_TYPE_LCD_MODE_CHANGE = 0,
  MSG_TYPE_LED_MODE_CHANGE,
  MSG_TYPE_SYSTEM_STATUS
} Message_Type_t;

typedef struct {
  Message_Type_t type;
  union {
    LCD_Mode_t lcd_mode;
    LED_Mode_t led_mode;
    uint32_t system_status;
  } data;
  uint32_t timestamp;
} System_Message_t;

typedef struct {
  uint32_t frame_times[FPS_SAMPLE_COUNT];  // Ring buffer for frame times
  uint8_t current_index;                   // Current index in ring buffer
  uint8_t sample_count;                    // Number of valid samples
  uint32_t last_frame_time;                // Time of last frame
  float current_fps;                       // Current calculated FPS
  uint32_t frame_counter;                  // Total frame counter
  int last_displayed_fps;                  // Last FPS value displayed on screen
} FPS_Calculator_t;

// Smart refresh system for LCD optimization
typedef struct {
  uint32_t last_frame_hash;               // Hash of last rendered frame
  uint32_t last_animation_frame;          // Last animation frame number
  LCD_Mode_t last_active_mode;            // Last active LCD mode
  bool force_full_refresh;                // Force full screen refresh flag
  bool fps_area_dirty;                    // FPS display area needs refresh
  
  // Dirty regions tracking
  struct {
    uint16_t x, y, w, h;                  // Rectangle coordinates
    bool is_dirty;                        // Needs refresh
  } dirty_regions[8];                     // Track up to 8 dirty regions
  uint8_t dirty_count;                    // Number of dirty regions
  
  // Performance counters
  uint32_t full_refreshes;                // Count of full screen refreshes
  uint32_t partial_refreshes;             // Count of partial refreshes
  uint32_t skipped_frames;                // Count of skipped frames
} SmartRefresh_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_DEBOUNCE_TIME 50        // 50ms debounce time
#define LED_SLOW_BLINK_PERIOD 1000  // 1000ms period for slow blink
#define LED_FAST_BLINK_PERIOD 200   // 200ms period for fast blink
#define SD_CARD_DEBOUNCE_TIME 100   // 100ms debounce time for SD card detection

// FPS calculation constants
#define FPS_SAMPLE_COUNT 10         // Number of samples for FPS averaging
#define FPS_UPDATE_INTERVAL 5       // Update FPS display every N frames

// Smart refresh constants  
#define MAX_DIRTY_REGIONS 8         // Maximum number of dirty regions to track
#define MIN_REFRESH_AREA 64         // Minimum area (pixels) to consider for partial refresh
#define HASH_PRIME 31               // Prime number for hash calculation

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static LED_Mode_t current_led_mode = LED_MODE_OFF;
static LCD_Mode_t current_lcd_mode = LCD_MODE_OFF;
static Key_State_t last_key_state = KEY_STATE_RELEASED;
static SD_Card_State_t last_sd_card_state = SD_CARD_NOT_DETECTED;
static uint8_t sd_card_initialized = 0;

// Message queues
static osMessageQueueId_t lcd_message_queue;
static osMessageQueueId_t led_message_queue;

// FPS calculator instance
static FPS_Calculator_t fps_calc = {0};

// Smart refresh system instance
static SmartRefresh_t smart_refresh = {0};
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
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lcdTask */
osThreadId_t lcdTaskHandle;
const osThreadAttr_t lcdTask_attributes = {
  .name = "lcdTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartKeyTask(void *argument);
void StartLedTask(void *argument);
void StartSDCardTask(void *argument);
void StartLCDTask(void *argument);

// FPS calculation functions
void FPS_Init(FPS_Calculator_t *fps);
void FPS_UpdateFrame(FPS_Calculator_t *fps);
float FPS_GetCurrent(FPS_Calculator_t *fps);
void FPS_DisplayOnLCD(FPS_Calculator_t *fps, int x, int y);

// Smart refresh functions
void SmartRefresh_Init(SmartRefresh_t *sr);
uint32_t SmartRefresh_CalculateFrameHash(LCD_Mode_t mode, uint32_t frame, uint8_t *colors, uint8_t color_count);
bool SmartRefresh_ShouldUpdate(SmartRefresh_t *sr, LCD_Mode_t mode, uint32_t frame);
void SmartRefresh_AddDirtyRegion(SmartRefresh_t *sr, uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void SmartRefresh_ClearDirtyRegions(SmartRefresh_t *sr);
void SmartRefresh_ForceFullRefresh(SmartRefresh_t *sr);
bool FPS_ShouldUpdateDisplay(FPS_Calculator_t *fps);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartKeyTask(void *argument);
void StartLedTask(void *argument);
void StartSDCardTask(void *argument);
void StartLCDTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
  called if a stack overflow is detected. */
}
/* USER CODE END 4 */

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
  // Create message queues
  lcd_message_queue = osMessageQueueNew(5, sizeof(System_Message_t), NULL);
  led_message_queue = osMessageQueueNew(5, sizeof(System_Message_t), NULL);

  if (lcd_message_queue == NULL || led_message_queue == NULL) {
    // Queue creation failed - handle error
    Error_Handler();
  }
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

  /* creation of lcdTask */
  lcdTaskHandle = osThreadNew(StartLCDTask, NULL, &lcdTask_attributes);

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
    osDelay(1000);  // Run every 1 second
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
  System_Message_t message;

  for (;;) {
    // Read current key state (assuming active low)
    current_key_state =
        (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
            ? KEY_STATE_PRESSED
            : KEY_STATE_RELEASED;

    // Check for key press (rising edge from released to pressed)
    if (current_key_state == KEY_STATE_PRESSED &&
        last_key_state == KEY_STATE_RELEASED) {
      // Calculate next modes
      LED_Mode_t next_led_mode = (current_led_mode + 1) % LED_MODE_COUNT;
      LCD_Mode_t next_lcd_mode = (current_lcd_mode + 1) % LCD_MODE_COUNT;

      // Send LED mode change message
      message.type = MSG_TYPE_LED_MODE_CHANGE;
      message.data.led_mode = next_led_mode;
      message.timestamp = osKernelGetTickCount();

      osStatus_t status = osMessageQueuePut(led_message_queue, &message, 0, 0);
      if (status == osOK) {
        current_led_mode = next_led_mode;
      }

      // Send LCD mode change message
      message.type = MSG_TYPE_LCD_MODE_CHANGE;
      message.data.lcd_mode = next_lcd_mode;
      message.timestamp = osKernelGetTickCount();

      status = osMessageQueuePut(lcd_message_queue, &message, 0, 0);
      if (status == osOK) {
        current_lcd_mode = next_lcd_mode;
      }

      // Wait for debounce
      osDelay(KEY_DEBOUNCE_TIME);
    }

    last_key_state = current_key_state;
    osDelay(10);  // Check key every 10ms
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
  System_Message_t message;
  LED_Mode_t active_led_mode = LED_MODE_OFF;

  for (;;) {
    // Check for mode change messages (non-blocking)
    osStatus_t status = osMessageQueueGet(led_message_queue, &message, NULL, 0);
    if (status == osOK && message.type == MSG_TYPE_LED_MODE_CHANGE) {
      active_led_mode = message.data.led_mode;
      led_counter = 0;  // Reset counter on mode change
    }

    switch (active_led_mode) {
      case LED_MODE_OFF:
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        osDelay(100);
        break;

      case LED_MODE_SLOW_BLINK:
        led_counter++;
        if (led_counter >= (LED_SLOW_BLINK_PERIOD / 100)) {
          led_state =
              (led_state == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
          HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, led_state);
          led_counter = 0;
        }
        osDelay(100);
        break;

      case LED_MODE_FAST_BLINK:
        led_counter++;
        if (led_counter >= (LED_FAST_BLINK_PERIOD / 100)) {
          led_state =
              (led_state == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
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
        active_led_mode = LED_MODE_OFF;
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

/* USER CODE BEGIN Header_StartLCDTask */
/**
 * @brief Function implementing the lcdTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLCDTask */
void StartLCDTask(void *argument)
{
  /* USER CODE BEGIN StartLCDTask */
  (void)argument;

  // Initialize LCD
  LCD_Init();

  // Initialize FPS calculator
  FPS_Init(&fps_calc);
  
  // Initialize Smart Refresh system
  SmartRefresh_Init(&smart_refresh);

  // 定义颜色数组
  uint16_t colors[] = {RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, WHITE, BLACK};
  uint8_t color_count = sizeof(colors) / sizeof(colors[0]);

  LCD_Mode_t active_lcd_mode = LCD_MODE_OFF;
  LCD_Mode_t last_lcd_mode = LCD_MODE_COUNT;  // Force initial update
  uint32_t animation_frame = 0;
  System_Message_t message;

  // LCD animation and display loop
  for (;;) {
    bool fps_needs_update = false;  // Declare at loop scope
    
    // Check for mode change messages with timeout for animation update
    osStatus_t status =
        osMessageQueueGet(lcd_message_queue, &message, NULL, 50);
    if (status == osOK && message.type == MSG_TYPE_LCD_MODE_CHANGE) {
      active_lcd_mode = message.data.lcd_mode;
      // Force refresh when mode changes
      SmartRefresh_ForceFullRefresh(&smart_refresh);
    }

    // Check if mode changed, reset animation frame
    if (active_lcd_mode != last_lcd_mode) {
      animation_frame = 0;
      last_lcd_mode = active_lcd_mode;
    }
    
    // Check if frame needs updating using smart refresh
    bool should_render = SmartRefresh_ShouldUpdate(&smart_refresh, active_lcd_mode, animation_frame);
    
    // Check if FPS display needs updating
    fps_needs_update = (fps_calc.frame_counter % FPS_UPDATE_INTERVAL == 0) && FPS_ShouldUpdateDisplay(&fps_calc);
    if (fps_needs_update) {
      smart_refresh.fps_area_dirty = true;
      should_render = true;
    }
    
    // Skip rendering if no update needed
    if (!should_render) {
      // Update FPS calculation even if we skip rendering
      FPS_UpdateFrame(&fps_calc);
      continue;
    }

    switch (active_lcd_mode) {
      case LCD_MODE_OFF:
        LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
        // No additional delay needed as osMessageQueueGet provides timeout
        break;

      case LCD_MODE_COLOR_BLOCKS:
        // 彩色方块动画
        {
          uint8_t color_idx = (animation_frame / 2) % color_count;
          LCD_Fill(0, 0, LCD_W, LCD_H, colors[color_idx]);
          animation_frame++;
        }
        break;

      case LCD_MODE_GRADIENT_STRIPES:
        // 渐变条纹动画
        {
          for (int y = 0; y < LCD_H; y += 4) {
            uint16_t color_idx = (animation_frame + y / 4) % color_count;
            LCD_Fill(0, y, LCD_W, y + 4, colors[color_idx]);
          }
          animation_frame++;
        }
        break;

      case LCD_MODE_CONCENTRIC_SQUARES:
        // 同心方块动画
        {
          LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);

          int size = ((animation_frame % 60) * 4);
          if (size > LCD_W / 2)
            size = LCD_W - ((animation_frame % 60) - LCD_W / 8) * 4;

          if (size > 0) {
            int x = (LCD_W - size) / 2;
            int y = (LCD_H - size) / 2;

            uint16_t color = colors[(animation_frame / 5) % color_count];
            LCD_Fill(x, y, x + size, y + size, color);
          }

          animation_frame++;
        }
        break;

      case LCD_MODE_COLOR_GRID:
        // 彩色网格动画 (slower update every 3 frames)
        {
          if (animation_frame % 3 == 0) {
            for (int x = 0; x < LCD_W; x += 20) {
              for (int y = 0; y < LCD_H; y += 20) {
                uint16_t color =
                    colors[(x / 20 + y / 20 + animation_frame / 3) %
                           color_count];
                LCD_Fill(x, y, x + 20, y + 20, color);
              }
            }
          }
          animation_frame++;
        }
        break;

      case LCD_MODE_RIPPLE_CIRCLES:
        // 圆形波纹动画
        {
          LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);

          int center_x = LCD_W / 2;
          int center_y = LCD_H / 2;

          for (int radius = 5; radius < 120; radius += 15) {
            int animated_radius = radius + (animation_frame * 2) % 30 - 15;
            if (animated_radius > 5 && animated_radius < 120) {
              uint16_t color =
                  colors[(radius / 15 + animation_frame / 5) % color_count];
              Draw_Circle(center_x, center_y, animated_radius, color);
            }
          }
          animation_frame++;
        }
        break;

      case LCD_MODE_TEXT_DISPLAY:
        // 文字显示动画 (slower update every 4 frames)
        {
          if (animation_frame % 4 == 0) {
            LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);

            const char *text = "TKX LCD";
            int text_x = 60;
            int text_y = 100;

            // Show characters one by one based on animation frame
            int text_len = 7;  // Length of "TKX LCD"
            int chars_to_show = (animation_frame / 20) %
                                (text_len + 5);  // +5 for pause between cycles

            for (int i = 0; i < chars_to_show && i < text_len; i++) {
              uint16_t color = colors[i % color_count];
              LCD_ShowChar(text_x + i * 24, text_y, text[i], color, BLACK, 32,
                           0);
            }
          }

          animation_frame++;
        }
        break;

      default:
        active_lcd_mode = LCD_MODE_OFF;
        break;
    }

    // Update performance counters
    if (smart_refresh.force_full_refresh || (active_lcd_mode != LCD_MODE_OFF)) {
      smart_refresh.full_refreshes++;
    } else if (smart_refresh.fps_area_dirty) {
      smart_refresh.partial_refreshes++;
    }
    
    // Clear dirty regions after rendering
    SmartRefresh_ClearDirtyRegions(&smart_refresh);
    
    // Update FPS calculation
    FPS_UpdateFrame(&fps_calc);

    // Display FPS in top-right corner (only when needed)
    if (smart_refresh.fps_area_dirty || fps_needs_update) {
      FPS_DisplayOnLCD(&fps_calc, LCD_W - 72, 5);  // Position at top-right
    }
    
    // Increment animation frame for next iteration
    animation_frame++;
  }
  /* USER CODE END StartLCDTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// Initialize Smart Refresh system
void SmartRefresh_Init(SmartRefresh_t *sr) {
  sr->last_frame_hash = 0;
  sr->last_animation_frame = 0;
  sr->last_active_mode = LCD_MODE_COUNT; // Force initial refresh
  sr->force_full_refresh = true;
  sr->fps_area_dirty = false;
  sr->dirty_count = 0;
  sr->full_refreshes = 0;
  sr->partial_refreshes = 0;
  sr->skipped_frames = 0;
  
  // Clear all dirty regions
  for(int i = 0; i < MAX_DIRTY_REGIONS; i++) {
    sr->dirty_regions[i].is_dirty = false;
  }
}

// Calculate a simple hash for the current frame state
uint32_t SmartRefresh_CalculateFrameHash(LCD_Mode_t mode, uint32_t frame, uint8_t *colors, uint8_t color_count) {
  uint32_t hash = 0;
  
  // Hash the mode
  hash = hash * HASH_PRIME + (uint32_t)mode;
  
  // Hash the frame number (with some dampening for smooth animations)
  switch(mode) {
    case LCD_MODE_COLOR_BLOCKS:
      hash = hash * HASH_PRIME + (frame / 2);  // Changes every 2 frames
      break;
    case LCD_MODE_GRADIENT_STRIPES:
      hash = hash * HASH_PRIME + frame;        // Changes every frame
      break;
    case LCD_MODE_CONCENTRIC_SQUARES:
      hash = hash * HASH_PRIME + (frame % 60); // 60-frame cycle
      break;
    case LCD_MODE_COLOR_GRID:
      hash = hash * HASH_PRIME + (frame / 3);  // Changes every 3 frames
      break;
    case LCD_MODE_RIPPLE_CIRCLES:
      hash = hash * HASH_PRIME + frame;        // Changes every frame
      break;
    case LCD_MODE_TEXT_DISPLAY:
      hash = hash * HASH_PRIME + (frame / 20); // Changes every 20 frames
      break;
    case LCD_MODE_OFF:
    default:
      hash = hash * HASH_PRIME + 0;            // Static
      break;
  }
  
  // Add color count to hash (for variety)
  hash = hash * HASH_PRIME + color_count;
  
  return hash;
}

// Check if frame needs to be updated
bool SmartRefresh_ShouldUpdate(SmartRefresh_t *sr, LCD_Mode_t mode, uint32_t frame) {
  // Force refresh if mode changed
  if(mode != sr->last_active_mode) {
    sr->force_full_refresh = true;
    sr->last_active_mode = mode;
    return true;
  }
  
  // Force refresh if explicitly requested
  if(sr->force_full_refresh) {
    return true;
  }
  
  // Calculate current frame hash
  uint8_t colors[] = {1, 2, 3, 4, 5, 6, 7, 8}; // Dummy colors for hash
  uint32_t current_hash = SmartRefresh_CalculateFrameHash(mode, frame, colors, 8);
  
  // Check if frame content has changed
  if(current_hash != sr->last_frame_hash) {
    sr->last_frame_hash = current_hash;
    sr->last_animation_frame = frame;
    return true;
  }
  
  // Check if we have any dirty regions (like FPS updates)
  if(sr->dirty_count > 0 || sr->fps_area_dirty) {
    return true;
  }
  
  // No update needed
  sr->skipped_frames++;
  return false;
}

// Add a dirty region for partial refresh
void SmartRefresh_AddDirtyRegion(SmartRefresh_t *sr, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  // Only add if area is significant enough
  if(w * h < MIN_REFRESH_AREA) {
    return;
  }
  
  // Find an empty slot or merge with existing region
  for(int i = 0; i < MAX_DIRTY_REGIONS; i++) {
    if(!sr->dirty_regions[i].is_dirty) {
      sr->dirty_regions[i].x = x;
      sr->dirty_regions[i].y = y;
      sr->dirty_regions[i].w = w;
      sr->dirty_regions[i].h = h;
      sr->dirty_regions[i].is_dirty = true;
      sr->dirty_count++;
      return;
    }
  }
  
  // If no empty slot, force full refresh
  sr->force_full_refresh = true;
}

// Clear all dirty regions after refresh
void SmartRefresh_ClearDirtyRegions(SmartRefresh_t *sr) {
  for(int i = 0; i < MAX_DIRTY_REGIONS; i++) {
    sr->dirty_regions[i].is_dirty = false;
  }
  sr->dirty_count = 0;
  sr->fps_area_dirty = false;
  sr->force_full_refresh = false;
}

// Force a full screen refresh
void SmartRefresh_ForceFullRefresh(SmartRefresh_t *sr) {
  sr->force_full_refresh = true;
  SmartRefresh_ClearDirtyRegions(sr);
}

// Initialize FPS calculator
void FPS_Init(FPS_Calculator_t *fps) {
  for (int i = 0; i < FPS_SAMPLE_COUNT; i++) {
    fps->frame_times[i] = 0;
  }
  fps->current_index = 0;
  fps->sample_count = 0;
  fps->last_frame_time = osKernelGetTickCount();
  fps->current_fps = 0.0f;
  fps->frame_counter = 0;
  fps->last_displayed_fps = -1;  // Force initial display
}

// Update frame timing and calculate FPS
void FPS_UpdateFrame(FPS_Calculator_t *fps) {
  uint32_t current_time = osKernelGetTickCount();
  uint32_t frame_time = current_time - fps->last_frame_time;

  // Store frame time in ring buffer
  fps->frame_times[fps->current_index] = frame_time;
  fps->current_index = (fps->current_index + 1) % FPS_SAMPLE_COUNT;

  if (fps->sample_count < FPS_SAMPLE_COUNT) {
    fps->sample_count++;
  }

  // Calculate average frame time
  uint32_t total_time = 0;
  for (int i = 0; i < fps->sample_count; i++) {
    total_time += fps->frame_times[i];
  }

  if (total_time > 0) {
    float avg_frame_time = (float)total_time / fps->sample_count;
    fps->current_fps = 1000.0f / avg_frame_time;  // Convert ms to FPS
  }

  fps->last_frame_time = current_time;
  fps->frame_counter++;
}

// Get current FPS
float FPS_GetCurrent(FPS_Calculator_t *fps) { return fps->current_fps; }

// Check if FPS display needs to be updated
bool FPS_ShouldUpdateDisplay(FPS_Calculator_t *fps) {
  int fps_int = (int)(fps->current_fps + 0.5f);
  return (fps_int != fps->last_displayed_fps);
}

// Display FPS on LCD (optimized - only update when value changes)
void FPS_DisplayOnLCD(FPS_Calculator_t *fps, int x, int y) {
  int fps_int = (int)(fps->current_fps + 0.5f);  // Round to nearest int

  // Only update display if FPS value changed
  if (fps_int == fps->last_displayed_fps) {
    return;
  }

  // Clear previous FPS area (approximate width: 72 pixels)
  LCD_Fill(x, y, x + 72, y + 16, BLACK);

  // Create FPS string
  char fps_str[16];

  // Format: "FPS:XX"
  fps_str[0] = 'F';
  fps_str[1] = 'P';
  fps_str[2] = 'S';
  fps_str[3] = ':';

  // Convert number to string (simple implementation)
  if (fps_int >= 100) {
    fps_str[4] = '0' + (fps_int / 100);
    fps_str[5] = '0' + ((fps_int % 100) / 10);
    fps_str[6] = '0' + (fps_int % 10);
    fps_str[7] = '\0';
  } else if (fps_int >= 10) {
    fps_str[4] = '0' + (fps_int / 10);
    fps_str[5] = '0' + (fps_int % 10);
    fps_str[6] = '\0';
  } else {
    fps_str[4] = '0' + fps_int;
    fps_str[5] = '\0';
  }

  // Display on LCD with small font
  for (int i = 0; fps_str[i] != '\0'; i++) {
    LCD_ShowChar(x + i * 12, y, fps_str[i], WHITE, BLACK, 16, 0);
  }

  // Update last displayed value
  fps->last_displayed_fps = fps_int;
}

/* USER CODE END Application */

