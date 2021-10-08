// Includes --------------------------------------------------------------------
#include "aim.h"
#include "usart.h"
#include "stm32f3xx_hal_uart.h"
#include <stdio.h>
#include "logger.h"

// Private Typedef -------------------------------------------------------------

// Private Macro ---------------------------------------------------------------

// Private Variables -----------------------------------------------------------
uint8_t rxdata[256] = {0};

// Private Function prototypes -------------------------------------------------
char str[80];

// Functions -------------------------------------------------------------------
uint16_t AimReadData(AimData_t *data)
{
  uint8_t txdata[7] = "?DATD\r\n";

  for (uint32_t i = 0; i < 256; i++)
  {
    rxdata[i] = 0;
  }
  float dist, elev, tilt;

  __HAL_UART_FLUSH_RDR(&huart3);
  HAL_UART_Transmit(&huart3, txdata, sizeof(txdata), 1000);
  HAL_UART_Receive(&huart3, rxdata, sizeof(rxdata), 100);

  uint16_t end_pos = 0;
  while (rxdata[end_pos++] != 0)
  {
    ;
  }
  if (end_pos < 4)
  {
    platformLog("\r\nAim not response\r\n");
    return 1;
  }
  else
  {
    platformLog("\r\nAim response:\r\n");
    HAL_UART_Transmit(&huart2, rxdata, end_pos, 1000);    
  }

  int16_t err = sscanf((char *)rxdata, "%*s %*s %f %f %f", &dist, &elev, &tilt);
  if (err != 3)
  {
    platformLog("uncorrect format:\r\n");
    return 2;
  } 

  data->dist = (int32_t)(dist * 1000);
  data->elev = (int32_t)(elev * 1000);
  data->tilt = (int32_t)(tilt * 1000);

  uint32_t cnt = sprintf(str, "distance: %ld; elevation: %ld; tilt: %ld\r\n\r\n", data->dist, data->elev, data->tilt);
  HAL_UART_Transmit(&huart2, (uint8_t *)str, cnt, 1000);

  return 0;
}
