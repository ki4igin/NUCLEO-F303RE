/******************************************************************************
  * \attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        www.st.com/myliberty
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/

/*! \file
 *
 *  \author 
 *
 *  \brief UART communication handling implementation.
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "usart.h"
#include "st_errno.h"

#define USART_TIMEOUT 1000

UART_HandleTypeDef *pUsart = 0;
UART_HandleTypeDef  huart2;
UART_HandleTypeDef  huart3;

/* USART2 init function */
void MX_USART2_UART_Init(void)
{
  

  huart2.Instance          = USART2;
  huart2.Init.BaudRate     = 115200;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

void MX_USART3_UART_Init(void)
{
  huart3.Instance                    = USART3;
  huart3.Init.BaudRate               = 115200;
  huart3.Init.WordLength             = UART_WORDLENGTH_8B;
  huart3.Init.StopBits               = UART_STOPBITS_1;
  huart3.Init.Parity                 = UART_PARITY_NONE;
  huart3.Init.Mode                   = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  This function initalize the UART handle.
	* @param	husart : already initalized handle to USART HW
  * @retval none :
  */
void UsartInit(UART_HandleTypeDef *husart)
{
  pUsart = husart;
}

/**
  * @brief  This function Transmit one data byte via USART
	* @param	data : data to be transmitted
  * @retval ERR_INVALID_HANDLE : in case the SPI HW is not initalized yet
  * @retval others : HAL status
  */
uint8_t UsartTxByte(uint8_t data)
{
  if (pUsart == 0)
    return ERR_INVALID_HANDLE;

  return HAL_UART_Transmit(pUsart, &data, 1, USART_TIMEOUT);
}

/**
  * @brief  This function Transmit data via USART
	* @param	data : data to be transmitted
	* @param	dataLen : length of data to be transmitted
  * @retval ERR_INVALID_HANDLE : in case the SPI HW is not initalized yet
  * @retval others : HAL status
  */
uint8_t UsartTx(uint8_t *data, uint16_t dataLen)
{
  if (pUsart == 0)
    return ERR_INVALID_HANDLE;

  return HAL_UART_Transmit(pUsart, data, dataLen, USART_TIMEOUT);
}

/**
  * @brief  This function Receive data via USART
	* @param	data : data where received data shall be stored
	* @param	dataLen : length of received data
  * @retval ERR_INVALID_HANDLE : in case the SPI HW is not initalized yet
  * @retval others : HAL status
  */
uint8_t UsartRx(uint8_t *data, uint16_t *dataLen)
{
  uint8_t err = ERR_NONE;

  if (pUsart == 0)
    return ERR_INVALID_HANDLE;

  for (uint8_t i = 0; i < *dataLen; i++)
  {
    err |= HAL_UART_Receive(pUsart, &data[i], 1, USART_TIMEOUT);
    if (data[i] == 0)
    {
      *dataLen = i;
    }
  }
  return err;
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (uartHandle->Instance == USART2)
  {
    /* USER CODE BEGIN USART2_MspInit 0 */

    /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin       = USART_TX_Pin | USART_RX_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    /* USER CODE BEGIN USART2_MspInit 1 */

    /* USER CODE END USART2_MspInit 1 */
  }
  else if (uartHandle->Instance == USART3)
  {
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
