/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "lcd_init.h"
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
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void LCD_ShowAnimation(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// LCD动画演示函数
void LCD_ShowAnimation(void) {
    // 定义颜色数组
    uint16_t colors[] = {RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, WHITE, BLACK};
    uint8_t color_count = sizeof(colors) / sizeof(colors[0]);
    
    // 彩色方块动画
    for(int i = 0; i < 3; i++) {
        for(uint8_t c = 0; c < color_count; c++) {
            LCD_Fill(0, 0, LCD_W, LCD_H, colors[c]);
            HAL_Delay(300);
        }
    }
    
    // 渐变条纹动画
    for(int frame = 0; frame < 50; frame++) {
        for(int y = 0; y < LCD_H; y += 4) {
            uint16_t color = (frame + y/4) % color_count;
            LCD_Fill(0, y, LCD_W, y + 4, colors[color]);
        }
        HAL_Delay(50);
    }
    
    // 同心方块动画
    for(int frame = 0; frame < 30; frame++) {
        LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
        
        int size = frame * 4;
        if(size > LCD_W/2) size = LCD_W - (frame - LCD_W/8) * 4;
        
        int x = (LCD_W - size) / 2;
        int y = (LCD_H - size) / 2;
        
        uint16_t color = colors[frame % color_count];
        LCD_Fill(x, y, x + size, y + size, color);
        
        HAL_Delay(100);
    }
    
    // 彩色网格动画
    for(int frame = 0; frame < 20; frame++) {
        for(int x = 0; x < LCD_W; x += 20) {
            for(int y = 0; y < LCD_H; y += 20) {
                uint16_t color = colors[(x/20 + y/20 + frame) % color_count];
                LCD_Fill(x, y, x + 20, y + 20, color);
            }
        }
        HAL_Delay(150);
    }
    
    // 圆形波纹动画
    for(int frame = 0; frame < 40; frame++) {
        LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
        
        int center_x = LCD_W / 2;
        int center_y = LCD_H / 2;
        
        for(int radius = 5; radius < 120; radius += 15) {
            int animated_radius = radius + (frame * 2) % 30 - 15;
            if(animated_radius > 5 && animated_radius < 120) {
                uint16_t color = colors[(radius/15 + frame/5) % color_count];
                Draw_Circle(center_x, center_y, animated_radius, color);
            }
        }
        HAL_Delay(80);
    }
    
    // 文字显示动画
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    
    // 显示彩色文字
    const char* text = "TKX LCD";
    int text_x = 60;
    int text_y = 100;
    
    for(int i = 0; text[i] != '\0'; i++) {
        uint16_t color = colors[i % color_count];
        LCD_ShowChar(text_x + i * 24, text_y, text[i], color, BLACK, 32, 0);
        HAL_Delay(200);
    }
    
    HAL_Delay(2000);
    
    // 最终显示蓝色
    LCD_Fill(0, 0, LCD_W, LCD_H, BLUE);
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  // 初始化LCD
  LCD_Init();
  
  // 显示动画
  LCD_ShowAnimation();
  
  HAL_SD_DeInit(&hsd);  // 先反初始化

  // 按照正确的流程重新配置并初始化
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  if (HAL_SD_Init(&hsd) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  __asm volatile("BKPT #0");
  while (1) {
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
