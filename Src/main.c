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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "bsp_pwm.h"
#include "bsp_usart.h"
#include "struct_typedef.h"
#include <stdarg.h>
#include <time.h>
#include <stdio.h>
#include "bsp_buzzer.h"
#include "bsp_rc.h"
#include <string.h>
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
#define MAX_PSC                 1000

#define MAX_BUZZER_PWM      20000
#define MIN_BUZZER_PWM      10000

uint16_t psc1 = 1;
uint16_t psc2 = 1000;


uint16_t pwm = MIN_BUZZER_PWM;



void usart_printf(const char *fmt,...)//用于打印
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);

    
    len = vsprintf((char *)tx_buf, fmt, ap);

    va_end(ap);

    usart2_tx_dma_enable(tx_buf, len);//用DMA串口发送出去；

}
uint8_t exit_flag = 0;//判断进不进中断
uint8_t rising_falling_flag;//读取按键状态



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//在外部中断函数里写要干的事，判断按键状态；
{
  if(GPIO_Pin==KEY_Pin)//判断是不是KEY这个GPIO口；
  {
     if(exit_flag==0)
      {
            exit_flag = 1;
            rising_falling_flag = HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin);//把按键的状态赋值给rising_falling_flat;
      }
  }

}

uint8_t rx_buffer[50]; //接收缓冲区
uint8_t rx_index = 0;  //索引
char robomaster_str[11]; //放robomaster
int number_value;   //放数字

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //回调函数被自动调用
{
    if (huart == &huart2)
    {
        rx_buffer[rx_index++] = huart2.Instance->DR; //将接收到的字节数据存储到 rx_buffer 数组中，并将 rx_index 自增1
        
        if (rx_index >= 13 && rx_buffer[0] == '#' && strncmp((char *)rx_buffer + 1, "ROBOMASTER", 10) == 0)//条件判断
        {
            strncpy(robomaster_str, (char *)rx_buffer + 1, 10);//复制robomaster到数组里
            robomaster_str[10] = '\0';//添加结束位
            sscanf((char *)rx_buffer + 11, "%d", &number_value);//把数字25移到整型变量
            rx_index = 0;
        }
        HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
    }
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
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  usart2_tx_dma_init();

  /* USER CODE BEGIN 2 */
  //开启定时器
    HAL_TIM_Base_Start(&htim5);
    //start pwm channel
    //开启PWM通道
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)==GPIO_PIN_SET)
    //没有按下按键，即为空闲状态；
    {
      aRGB_led_show(0xFF00FF00);  //纯绿灯；
      HAL_Delay(500);
      aRGB_led_show(0xFF000000);  //灭；
        HAL_Delay(500);
    }

    else if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)==GPIO_PIN_RESET) //如果按键按下；
    {
      int i;
      for(i=0;i<3;i++)
      {
        time_t timestamp = time(NULL); //获得时间戳；
        struct tm *time_info = localtime(&timestamp); //转化为现在的时间；
        char buffer[50];
         strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", time_info);  //使用strftime函数将struct tm结构体格式化为指定格式的字符串
         char final_str[100];
        sprintf(final_str, "ZZZ %s", buffer);
         usart_printf(final_str); //发送；
         aRGB_led_show(0xFF00FFFF); //橙色灯；
         HAL_Delay(500);
         //在进程中进入中断，快闪红灯；
         if(exit_flag == 1)//判断进入中断
         {
          exit_flag = 2;//避免反复进入；
         if( rising_falling_flag==GPIO_PIN_RESET)  //如果在三次的进程中，按下按键；
         {
           HAL_Delay(20); //消抖；
           if(rising_falling_flag==GPIO_PIN_RESET) //不是抖动；
           {
            HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin); //快闪红灯；
            exit_flag = 0; //重新中断；
           }
           else if(rising_falling_flag==GPIO_PIN_SET)  //是抖动；
           {
              exit_flag = 0;
           }
         }
         }
      }
      
        buzzer_on(psc1, pwm);//蜂鸣机；
         HAL_Delay(250);
        buzzer_on(psc2, pwm);
        HAL_Delay(250);

       }
       
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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
