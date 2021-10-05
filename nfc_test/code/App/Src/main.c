
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "rcc.h"
#include "platform.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "logger.h"
#include "st_errno.h"
#include "rfal_nfc.h"
#include "main.h"
#include "demo.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint8_t globalCommProtectCnt = 0;

uint32_t cmd = 0;

uint8_t buf[256] = {0};

/* Private function prototypes -----------------------------------------------*/

static void CmdWork(void);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();

  /* Initialize driver*/
  spiInit(&hspi1);

  /* Initialize log module */
  logUsartInit(&huart2);

  platformLog("Welcome to X-NUCLEO-NFC05A1\r\n");

  /* Infinite loop */

  /* Initalize RFAL */
  if (!demoIni())
  {
    /*
    * in case the rfal initalization failed signal it by flashing all LED
    * and stoping all operations
    */
    platformLog("Initialization failed..\r\n");
    while (1)
    {
      platformLedToogle(PLATFORM_LED_FIELD_PORT, PLATFORM_LED_FIELD_PIN);
      platformLedToogle(PLATFORM_LED_A_PORT, PLATFORM_LED_A_PIN);
      platformLedToogle(PLATFORM_LED_B_PORT, PLATFORM_LED_B_PIN);
      platformLedToogle(PLATFORM_LED_F_PORT, PLATFORM_LED_F_PIN);
      platformLedToogle(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);
      platformLedToogle(PLATFORM_LED_AP2P_PORT, PLATFORM_LED_AP2P_PIN);
      platformDelay(100);
    }
  }
  else
  {
    platformLog("Initialization succeeded..\r\n");
    for (int i = 0; i < 6; i++)
    {
      platformLedToogle(PLATFORM_LED_FIELD_PORT, PLATFORM_LED_FIELD_PIN);
      platformLedToogle(PLATFORM_LED_A_PORT, PLATFORM_LED_A_PIN);
      platformLedToogle(PLATFORM_LED_B_PORT, PLATFORM_LED_B_PIN);
      platformLedToogle(PLATFORM_LED_F_PORT, PLATFORM_LED_F_PIN);
      platformLedToogle(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);
      platformLedToogle(PLATFORM_LED_AP2P_PORT, PLATFORM_LED_AP2P_PIN);
      platformDelay(200);
    }

    platformLedOff(PLATFORM_LED_A_PORT, PLATFORM_LED_A_PIN);
    platformLedOff(PLATFORM_LED_B_PORT, PLATFORM_LED_B_PIN);
    platformLedOff(PLATFORM_LED_F_PORT, PLATFORM_LED_F_PIN);
    platformLedOff(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);
    platformLedOff(PLATFORM_LED_AP2P_PORT, PLATFORM_LED_AP2P_PIN);
    platformLedOff(PLATFORM_LED_FIELD_PORT, PLATFORM_LED_FIELD_PIN);
  }

  HAL_UART_Receive_IT(&huart2, (uint8_t *)&cmd, sizeof(cmd));

  while (1)
  {
    demoCycle();
    // rfalNfcWorker();
    if (huart2.RxXferCount == 0)
    {
      CmdWork();
      HAL_UART_Receive_IT(&huart2, (uint8_t *)&cmd, sizeof(cmd));
    }
    HAL_Delay(100);
  }
}

void CmdWork()
{
  switch (__REV(cmd))
  {
    case 0x626c696b:
      for (int i = 0; i < 6; i++)
      {
        platformLedToogle(PLATFORM_LED_FIELD_PORT, PLATFORM_LED_FIELD_PIN);
        platformLedToogle(PLATFORM_LED_A_PORT, PLATFORM_LED_A_PIN);
        platformLedToogle(PLATFORM_LED_B_PORT, PLATFORM_LED_B_PIN);
        platformLedToogle(PLATFORM_LED_F_PORT, PLATFORM_LED_F_PIN);
        platformLedToogle(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);
        platformLedToogle(PLATFORM_LED_AP2P_PORT, PLATFORM_LED_AP2P_PIN);
        platformDelay(200);
      }

      platformLedOff(PLATFORM_LED_A_PORT, PLATFORM_LED_A_PIN);
      platformLedOff(PLATFORM_LED_B_PORT, PLATFORM_LED_B_PIN);
      platformLedOff(PLATFORM_LED_F_PORT, PLATFORM_LED_F_PIN);
      platformLedOff(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);
      platformLedOff(PLATFORM_LED_AP2P_PORT, PLATFORM_LED_AP2P_PIN);
      platformLedOff(PLATFORM_LED_FIELD_PORT, PLATFORM_LED_FIELD_PIN);
      break;
    case 0x77727772:  //wrwr
      HAL_UART_Receive_IT(&huart2, (uint8_t *)&cmd, sizeof(cmd));
      while (huart2.RxXferCount > 0)
      {
        ;
      }
      uint32_t size = str2hex((char *)&cmd);
      platformLog("%s\r\n", hex2Str((unsigned char *)&size, 4));
      HAL_UART_Receive_IT(&huart2, buf, size);
      while (huart2.RxXferCount > 0)
      {
        ;
      }
      demowrData(buf, size);
      break;
    case 0x73746f70:  //stop
      demowrData(buf, 0);
      break;
    case 0x71776572:  // qwer
      NVIC_SystemReset();
      break;
    default:
      HAL_UART_Transmit(&huart2, (uint8_t *)&cmd, sizeof(cmd), 1000);
      break;
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  platformLog("%s, %d", file, line);
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
ex:
  printf("Wrong parameters value: file %s on line %d\r\n", file, line) * /
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
