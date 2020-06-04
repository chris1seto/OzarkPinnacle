#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stm32f4xx_hal.h>
#include "Watchdog.h"

static UART_HandleTypeDef uart_handle;

static uint8_t x_buffer[200];

#define MAX_TIMEOUT        2000

#define DRA818_CONNECT_RESPONSE_OK "+DMOCONNECT:0\r\n"
#define DRA818_CONNECT_COMMAND_SIZE    15
#define DRA818_CONNECT_RESPONSE_SIZE    15

#define DRA818_GROUP_LEN    48
#define DRA818_FILTER_LEN    20

// PA2/PA3
void Dra818_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  // Enable clocks
  __GPIOA_CLK_ENABLE();
  __USART2_CLK_ENABLE();

  // Configure GPIO
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;

  // TX
  GPIO_InitStruct.Pin       = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // RX
  GPIO_InitStruct.Pin       = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure the USART peripheral
  uart_handle.Instance          = USART2;
  uart_handle.Init.BaudRate     = 9600;
  uart_handle.Init.WordLength   = UART_WORDLENGTH_8B;
  uart_handle.Init.StopBits     = UART_STOPBITS_1;
  uart_handle.Init.Parity       = UART_PARITY_NONE;
  uart_handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  uart_handle.Init.Mode         = UART_MODE_TX_RX;

  // Commit the USART
  HAL_UART_Init(&uart_handle);
}

static bool BufferCompare(const uint8_t* a, const uint8_t* b, const uint32_t size)
{
  uint32_t i;

  for (i = 0; i < size; i++)
  {
    if (a[i] != b[i])
    {
      return false;
    }
  }

  return true;
}

bool Dra818_Connect(void)
{
  uint8_t command[DRA818_CONNECT_COMMAND_SIZE] = "AT+DMOCONNECT\r\n";
  uint8_t response[DRA818_CONNECT_RESPONSE_SIZE] = {0};

  // TX then RX
  HAL_UART_Transmit(&uart_handle, command, DRA818_CONNECT_COMMAND_SIZE, MAX_TIMEOUT);
  if (HAL_UART_Receive(&uart_handle, response, DRA818_CONNECT_RESPONSE_SIZE, MAX_TIMEOUT) != HAL_OK)
  {
    return false;
  }

  // Compare the response to an OK response
  if (!BufferCompare(response, (uint8_t*)DRA818_CONNECT_RESPONSE_OK, DRA818_CONNECT_RESPONSE_SIZE))
  {
    return false;
  }

  return true;
}

bool Dra818_SetGroup(const float txFreq, const float rxFreq, const char ctcssTx[4], const char ctcssRx[4], const uint8_t squelch)
{
  sprintf((char*)&x_buffer, "AT+DMOSETGROUP=0,%03.4f,%03.4f,%.4s,%c,%.4s\r\n", txFreq, rxFreq, ctcssTx, squelch + '0', ctcssRx);
  HAL_UART_Transmit(&uart_handle, x_buffer, DRA818_GROUP_LEN, MAX_TIMEOUT);
  return true;
}

bool Dra818_SetFilter(const uint8_t predeemphasis, const uint8_t highpass, const uint8_t lowpass)
{
  sprintf((char*)x_buffer, "AT+SETFILTER=%c,%c,%c\r\n", predeemphasis + '0', highpass + '0', lowpass + '0');
  HAL_UART_Transmit(&uart_handle, x_buffer, DRA818_FILTER_LEN, MAX_TIMEOUT);
  return true;
}