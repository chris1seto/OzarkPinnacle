/*
  Simple NMEA183 parser
    -- Chris Seto, 2018

  Resources
    * http://www.gpsinformation.org/dale/nmea.htm
*/
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stm32f4xx_hal.h>
#include "Queuex.h"
#include "Helpers.h"
#include "TokenIterate.h"
#include "Nmea0183.h"

// Sentence processors
static void ParseGsa(const uint32_t size);
static void ParseGga(const uint32_t size);
static void ParseRmc(const uint32_t size);
static void ParseZda(const uint32_t size);
static void ProcessSentence(const uint32_t size);

// Packet extractors
static uint8_t ExtractTime(TokenIterateT* t, NmeaTime_t* time);
static uint8_t ExtractDate(TokenIterateT* t, NmeaDate_t* date);
static uint8_t ExtractPosition(TokenIterateT* t, NmeaPosition_t* pos);
static uint8_t ExtractChar(TokenIterateT* t, uint8_t* c);
static uint8_t ExtractFloat(TokenIterateT* t, float* x);
static uint8_t ExtractInt(TokenIterateT* t, int32_t* x);

typedef struct
{
  enum NMEA_MESSAGE_TYPE type;
  char header[5];
  void(*processor)(const uint32_t);
  TickType_t last_message_time;
} NmeaProcessor_t;

// Task
static void Nmea0183Task(void * pvParameters);

// Serial DMA buffer
#define DMA_BUFFER_SIZE      1000
static uint8_t dma_buffer[DMA_BUFFER_SIZE];
static volatile uint32_t dma_header = 0;

// Nmea data queue
#define NMEA_BUFFER_SIZE    1000
static uint8_t nmea_buffer[NMEA_BUFFER_SIZE];
static struct QueueT nmea_queue;

// Message output queue
static QueueHandle_t nmea_message_queue;

#define NMEA_PACKET_BUFFER_SIZE  100
static uint8_t packet_buffer[NMEA_PACKET_BUFFER_SIZE];

#define PARSE_STATE_HEADER    0
#define PARSE_STATE_PAYLOAD    1
#define PARSE_STATE_CHECKSUM0  2
#define PARSE_STATE_CHECKSUM1  3
static uint32_t parser_state = PARSE_STATE_HEADER;

// NMEA processors
#define NMEA_PROCESSOR_COUNT  4
static NmeaProcessor_t processors[NMEA_PROCESSOR_COUNT] =
{
  {NMEA_MESSAGE_TYPE_GSA, "GPGSA", ParseGsa, 0},
  {NMEA_MESSAGE_TYPE_GGA, "GPGGA", ParseGga, 0},
  {NMEA_MESSAGE_TYPE_RMC, "GPRMC", ParseRmc, 0},
  {NMEA_MESSAGE_TYPE_ZDA, "GPZDA", ParseZda, 0}
};

// Peripheral handles
static UART_HandleTypeDef uart_handle;
static DMA_HandleTypeDef hdma_rx;

static void InitUart(void)
{
  // Init Uart
  GPIO_InitTypeDef GPIO_InitStruct;

  // Enable clocks
  __GPIOB_CLK_ENABLE();
  __USART3_CLK_ENABLE();
  __DMA1_CLK_ENABLE();

  // Configure GPIO
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure the USART peripheral
  uart_handle.Instance          = USART3;
  uart_handle.Init.BaudRate     = 9600;
  uart_handle.Init.WordLength   = UART_WORDLENGTH_8B;
  uart_handle.Init.StopBits     = UART_STOPBITS_1;
  uart_handle.Init.Parity       = UART_PARITY_NONE;
  uart_handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  uart_handle.Init.Mode         = UART_MODE_RX | UART_MODE_TX;

  // Commit the USART
  HAL_UART_Init(&uart_handle);

  // DMA USART3
  // RX: (DMA1, Stream 1, channel 4)
  // TX: (DMA1, Stream 3, Channel 4), (DMA1, Stream 4, Channel 7)
  hdma_rx.Instance                 = DMA1_Stream1;
  hdma_rx.Init.Channel             = DMA_CHANNEL_4;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_rx.Init.Mode                = DMA_CIRCULAR;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
  hdma_rx.Init.PeriphBurst         = DMA_PBURST_SINGLE;

  // Init DMA
  HAL_DMA_Init(&hdma_rx);

  // Link DMA
  __HAL_LINKDMA(&uart_handle, hdmarx, hdma_rx);

  // Start DMA
  HAL_UART_Receive_DMA(&uart_handle, (uint8_t*)&dma_buffer, DMA_BUFFER_SIZE);
}

UART_HandleTypeDef* Nmea0183_GetUartHandle(void)
{
  return &uart_handle;
}

uint8_t Nmea0183_Init(void)
{
  // Init RTOS queue
  nmea_message_queue = xQueueCreate(10, sizeof(GenericNmeaMessage_t));

  if (nmea_message_queue == NULL)
  {
    return 0;
  }

  QueueInit(&nmea_queue, nmea_buffer, NMEA_BUFFER_SIZE);

  // Start serial port
  InitUart();

  return 1;
}

void Nmea0183_StartParser(void)
{
  // Start Task
  xTaskCreate(Nmea0183Task,
    "Nmea0183ParserTask",
    1024,
    NULL,
    6,
    NULL);
}

// Verify a packet checksum
static uint8_t VerifyChecksum(const uint8_t x, const uint8_t* buffer, const uint32_t size)
{
  uint8_t checksum = 0;
  uint32_t i = 0;

  for (i = 0; i < size; i++)
  {
    checksum ^= buffer[i];
  }

  return (x == checksum);
}

static void CopyFromDma(void)
{
  uint32_t endIndex;

  // Get the index of the end of the buffer
  endIndex =  DMA_BUFFER_SIZE - DMA1_Stream1->NDTR;

  // If there is nothing to copy
  // If you let the buffer loop back, you will lose the entire buffer
  if (endIndex == dma_header)
  {
    return;
  }

  // Check if the buffer looped over
  if (endIndex < dma_header)
  {
    // Copy from the end of the dma buffer
    QueueAppendBuffer(&nmea_queue, dma_buffer + dma_header, DMA_BUFFER_SIZE - dma_header);

    // It has. We need to copy from the end of the circ buffer, then again from the start
    // Copy from the start of the dma buffer
    QueueAppendBuffer(&nmea_queue, dma_buffer, endIndex);
  }
  else
  {
    // Copy from the dma_header to the endindex
    QueueAppendBuffer(&nmea_queue, dma_buffer + dma_header, endIndex - dma_header);
  }

  // Relocate the DMA head to the end of where we just read
  dma_header = endIndex;
}

// Parse NMEA
static void Nmea0183Task(void * pvParameters)
{
  uint32_t parsePtr = 0;
  uint8_t frameChecksum;
  uint32_t bufferPtr;
  uint8_t workingByte;
  TickType_t xLastNmeaTime = 0;

  // Parse NMEA
  while (true)
  {
    // Block until it's time to start
    vTaskDelayUntil(&xLastNmeaTime, 10);

    // Import data into the queue from the DMA buffer
    CopyFromDma();

    // While there is data in the queue
    while (parsePtr < QueueCount(&nmea_queue))
    {
      // Get the current ptr
      workingByte = QueuePeek(&nmea_queue, parsePtr++);

      // State machine parser
      switch(parser_state)
      {
        // Look for and consume header byte $
        case PARSE_STATE_HEADER:
          if (workingByte == '$')
          {
            parser_state = PARSE_STATE_PAYLOAD;
          }

          // Dequeue whatever we get at this point, header or not
          QueueDequeqe(&nmea_queue, 1);
          parsePtr = 0;
          bufferPtr = 0;
          break;

        // Wait until we get past the payload
        case PARSE_STATE_PAYLOAD:
          if (workingByte == '*')
          {
            parser_state = PARSE_STATE_CHECKSUM0;
          }
          else
          {
            packet_buffer[bufferPtr++] = workingByte;
          }
          break;

        // First checksum byte
        case PARSE_STATE_CHECKSUM0:

          // Just collect this byte to start building the checksum
          frameChecksum = (Hex2int(workingByte) << 4);

          // Move on
          parser_state = PARSE_STATE_CHECKSUM1;
          break;

        // Second checksum byte
        case PARSE_STATE_CHECKSUM1 :
          // Continue building the checksum
          frameChecksum |= Hex2int(workingByte);

          // Attempt to verify the checksum
          if (VerifyChecksum(frameChecksum, packet_buffer, bufferPtr))
          {
            // Dequeue the entire packet if success
            QueueDequeqe(&nmea_queue, parsePtr);
            parsePtr = 0;

            // Process sentence
            ProcessSentence(bufferPtr);
          }

          // Reset
          parser_state = PARSE_STATE_HEADER;
          break;
      }
    }
  }
}

static uint8_t MatchUpTo(const uint8_t* x, const uint8_t* y, const uint32_t len)
{
  uint32_t i;

  for (i = 0; i < len; i++)
  {
    if (x[i] != y[i])
    {
      return 0;
    }
  }

  return 1;
}

static void ProcessSentence(const uint32_t size)
{
  uint32_t i;

  // Try to match to a known processor
  for (i = 0; i < NMEA_PROCESSOR_COUNT; i++)
  {
    // Check if we have a match
    if (MatchUpTo((uint8_t*)processors[i].header, packet_buffer, 5))
    {
      // Call the processor
      processors[i].processor(size);

      // Set the last update time
      processors[i].last_message_time = xTaskGetTickCount();

      return;
    }
  }
}

// Extract time
static uint8_t ExtractTime(TokenIterateT* t, NmeaTime_t* time)
{
  uint8_t* token;
  uint32_t token_size;

  // Get the token
  TokenIteratorForward(t, &token, &token_size);

  // Check size
  if (token_size < 6)
  {
    return 0;
  }

  // Hour
  time->hour = atoil(token + 0, 2);

  // Minute
  time->minute = atoil(token + 2, 2);

  // Second (might be a float)
  time->second = atofl(token + 4, token_size - 4);

  return 1;
}

// Extract date
static uint8_t ExtractDate(TokenIterateT* t, NmeaDate_t* date)
{
  uint8_t* token;
  uint32_t token_size;

  // Get the token
  TokenIteratorForward(t, &token, &token_size);

  // Check size
  if(token_size < 6)
  {
    return 0;
  }

  // Day
  date->day = atoil(token + 0, 2);

  // Month
  date->month = atoil(token + 2, 2);

  // year
  date->year = atofl(token + 4, token_size - 4);

  return 1;
}

// Extract position
static uint8_t ExtractPosition(TokenIterateT* t, NmeaPosition_t* pos)
{
  uint8_t* token;
  uint32_t token_size;

  int deg;
  float min;
  uint8_t dir = '-';

  // Get the token for lat
  TokenIteratorForward(t, &token, &token_size);

  // Check size
  if (token_size < 4)
  {
    return 0;
  }

  // Get the degrees
  deg = atoil(token, 2);

  // Get minutes
  min = atofl(token + 2, token_size - 2);

  // Get direction
  if (!ExtractChar(t, &dir))
  {
    return 0;
  }

  pos->lat = (deg + (min / 60.0)) * ((dir == 'N') ? 1 : -1);

  // Get the token for lon
  TokenIteratorForward(t, &token, &token_size);

  // Check size
  if (token_size < 4)
  {
    return 0;
  }

  // Get the degrees
  deg = atoil(token, 3);

  // Get minutes
  min = atofl(token + 3, token_size - 3);

  // Get direction
  if (!ExtractChar(t, &dir))
  {
    return 0;
  }

  pos->lon = (deg + (min / 60.0)) * ((dir == 'E') ? 1 : -1);

  return 1;
}


// Extract char
static uint8_t ExtractChar(TokenIterateT* t, uint8_t* c)
{
  uint8_t* token;
  uint32_t token_size;

  // Get the token
  TokenIteratorForward(t, &token, &token_size);

  // Check size
  if (token_size != 1)
  {
    *c = 0;
    return 0;
  }

  *c = token[0];

  return 1;
}

// Extract float
static uint8_t ExtractFloat(TokenIterateT* t, float* x)
{
  uint8_t* token;
  uint32_t token_size;

  // Get the token
  TokenIteratorForward(t, &token, &token_size);

  // Check size
  if (token_size == 0)
  {
    *x = 0;
    return 0;
  }

  *x = atofl(token, token_size);

  return 1;
}

// Extract int
static uint8_t ExtractInt(TokenIterateT* t, int32_t* x)
{
  uint8_t* token;
  uint32_t token_size;

  // Get the token
  TokenIteratorForward(t, &token, &token_size);

  // Check size
  if (token_size == 0)
  {
    *x = 0;
    return 0;
  }

  *x = atoil(token, token_size);

  return 1;
}

// Extract byte int
static uint8_t ExtractByteInt(TokenIterateT* t, int8_t* x)
{
  uint8_t* token;
  uint32_t token_size;

  // Get the token
  TokenIteratorForward(t, &token, &token_size);

  // Check size
  if (token_size == 0)
  {
    *x = 0;
    return 0;
  }

  *x = atoil(token, token_size);

  return 1;
}

//// Sentence Parsers ////

static void ParseGsa(const uint32_t size)
{
  /*
    $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39

    Where:
       GSA      Satellite status
       A        Auto selection of 2D or 3D fix (M = manual)
       3        3D fix - values include: 1 = no fix
                         2 = 2D fix
                         3 = 3D fix
       04,05... PRNs of satellites used for fix (space for 12)
       2.5      PDOP (dilution of precision)
       1.3      Horizontal dilution of precision (HDOP)
       2.1      Vertical dilution of precision (VDOP)
       *39      the checksum data, always begins with *

  */
}

static void ParseRmc(const uint32_t size)
{
  /*
  $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A

  Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   latitude 48 deg 07.038' N
     01131.000,E  longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
     *6A          The checksum data, always begins with *

  */

  GenericNmeaMessage_t msg;
  msg.message_type = NMEA_MESSAGE_TYPE_RMC;
  TokenIterateT t;
  TokenIteratorInit(&t, ',', packet_buffer + 6, size - 6);

  // Time
  ExtractTime(&t, &msg.rmc.time);

  // Fix
  ExtractChar(&t, &msg.rmc.fix);

  // Position
  ExtractPosition(&t, &msg.rmc.position);

  // Speed
  ExtractFloat(&t, &msg.rmc.speed);

  // Track
  ExtractFloat(&t, &msg.rmc.track);

  // Date
  ExtractDate(&t, &msg.rmc.date);

  // Try to queue it
  xQueueSendToBack(nmea_message_queue, &msg, 0);
}

static void ParseGga(const uint32_t size)
{
  /*
    $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47

    Where:
      GGA          Global Positioning System Fix Data
      123519       Fix taken at 12:35:19 UTC
      4807.038,N   latitude 48 deg 07.038' N
      01131.000,E  longitude 11 deg 31.000' E
      1            Fix quality: 0 = invalid
                  1 = GPS fix (SPS)
                  2 = DGPS fix
                  3 = PPS fix
            4 = Real Time Kinematic
            5 = Float RTK
                  6 = estimated (dead reckoning) (2.3 feature)
            7 = Manual input mode
            8 = Simulation mode
      08           Number of satellites being tracked
      0.9          Horizontal dilution of position
      545.4,M      Altitude, Meters, above mean sea level
      46.9,M       Height of geoid (mean sea level) above WGS84
              ellipsoid
      (empty field) time in seconds since last DGPS update
      (empty field) DGPS station ID number
      *47          the checksum data, always begins with *
  */

  GenericNmeaMessage_t msg;
  msg.message_type = NMEA_MESSAGE_TYPE_GGA;
  TokenIterateT t;
  TokenIteratorInit(&t, ',', packet_buffer + 6, size - 6);

  // Time
  ExtractTime(&t, &msg.gga.time);

  // Position
  ExtractPosition(&t, &msg.gga.position);

  // Fix
  ExtractChar(&t, &msg.gga.fix);

  // Sats tracked
  ExtractInt(&t, &msg.gga.sat_count);

  // HDOP
  ExtractFloat(&t, &msg.gga.h_dop);

  // Altitude
  ExtractFloat(&t, &msg.gga.altitude);

  // Altitude units
  ExtractChar(&t, &msg.gga.altitude_units);

  // Try to queue it
  xQueueSendToBack(nmea_message_queue, &msg, 0);
}

static void ParseZda(const uint32_t size)
{
  GenericNmeaMessage_t msg;
  msg.message_type = NMEA_MESSAGE_TYPE_ZDA;
  TokenIterateT t;
  TokenIteratorInit(&t, ',', packet_buffer + 6, size - 6);

  msg.zda.valid = true;

  // Time
  if (!ExtractTime(&t, &msg.zda.time))
  {
    msg.zda.valid = false;
  }

  // Day
  if (!ExtractByteInt(&t, (int8_t*)&msg.zda.day))
  {
    msg.zda.valid = false;
  }

  // Month
  if (!ExtractByteInt(&t, (int8_t*)&msg.zda.month))
  {
    msg.zda.valid = false;
  }

  // Year
  if (!ExtractInt(&t, (int32_t*)&msg.zda.year))
  {
    msg.zda.valid = false;
  }

  // Try to queue it
  xQueueSendToBack(nmea_message_queue, &msg, 0);
}

QueueHandle_t* Nmea0183_GetQueue(void)
{
  return &nmea_message_queue;
}