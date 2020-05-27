#include <stdio.h>
#include <string.h>
#include <stm32f4xx_hal.h>
#include "Nmea0183.h"
#include "Helpers.h"

#define CHECKSUM_SIZE   2

static uint8_t NmeaInsertChecksum(uint8_t* const buffer, const uint32_t size);

void UbloxNeo_Init(void)
{
}

static uint8_t NmeaInsertChecksum(uint8_t* const buffer, const uint32_t size)
{
  uint8_t checksum = 0;
  uint32_t i = 1;

  for (; i < size; i++)
  {
    // If we are at the end of the checksummed area
    if (buffer[i] == '*')
    {
      i++;
      break;
    }

    checksum ^= buffer[i];
  }

  // Check if we have enough space to insert the checksum
  if (i + CHECKSUM_SIZE > size)
  {
    return 0;
  }

  // Insert the checksum (high nibble first)
  buffer[i++] = Int2HexDigit((checksum >> 4) & 0x0f);
  buffer[i++] = Int2HexDigit(checksum & 0x0f);

  return 1;
}

// Set the output rate for a sentence
// UBX-13003221 - R15, page 130
#define CONFIG_SIZE 31
void UbloxNeo_SetOutputRate(const char* msg_id, const uint8_t rate)
{
  uint8_t config_msg[CONFIG_SIZE];

  // Format command
  sprintf((char*)config_msg, "$PUBX,40,%.3s,0,%i,0,0,0,0*XX\r\n", msg_id, rate);

  // Checksum
  NmeaInsertChecksum(config_msg, CONFIG_SIZE);

  // Send
  HAL_UART_Transmit(Nmea0183GetUartHandle(), config_msg, CONFIG_SIZE, 5000);
}
