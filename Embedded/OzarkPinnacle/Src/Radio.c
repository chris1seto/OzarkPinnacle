#include <stdio.h>
#include <stm32f4xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <string.h>
#include "Ax25.h"
#include "Dra818.h"
#include "Dra818Io.h"
#include "Audio.h"
#include "Afsk.h"
#include "Aprs.h"
#include "Beacon.h"
#include "Led.h"
#include "Nmea0183.h"
#include "Rtc.h"
#include "time.h"
#include "Radio.h"
#include "Watchdog.h"
#include "Log.h"

// Message queue
static QueueHandle_t tx_queue;
static QueueHandle_t rx_queue;

// Audio buffers
#define AUDIO_BUFFER_SIZE    22000
static uint8_t audio_out[AUDIO_BUFFER_SIZE];

#define AX25_BUFFER_SIZE  500
static uint8_t ax25_buffer[AX25_BUFFER_SIZE];

#define TX_RX_QUEUE_SIZE  10

static void RadioTask(void* pvParameters);
static void Dra818AprsInit(void);

#define PTT_DOWN_DELAY    50
#define PTT_UP_DELAY      20

const static char* TAG = "RADIO";

void Radio_Init(void)
{
  // Init queues
  tx_queue = xQueueCreate(TX_RX_QUEUE_SIZE, sizeof(RadioPacket_t));
  rx_queue = xQueueCreate(TX_RX_QUEUE_SIZE, sizeof(RadioPacket_t));
  
  // Init the radio
  Dra818AprsInit();
  
  // Create task
  xTaskCreate(RadioTask,
    TAG,
    2000,
    NULL,
    5,
    NULL);
}

static void Dra818AprsInit(void)
{
  // Bring up the module
  Dra818IoPowerUp();

  // Wait for the module to init
  WatchdogFeed();
  HAL_Delay(600);

  // Connect (to init the module?)
  LOG_I(TAG, "Trying to init Dra818... ");
  while (!Dra818_Connect())
  {
    LOG_W(TAG, "* [No Response]");
    WatchdogFeed();
    HAL_Delay(1000);
  }
  LOG_I(TAG, "- [Radio Init OK!]");

  // Configure group
  Dra818_SetGroup(144.390, 144.390, "0000", "0000", 8);
  WatchdogFeed();
  HAL_Delay(100);

  // Configure filter
  Dra818_SetFilter(0, 1, 1);

  // Set PA off for now
  Dra818IoSetLowRfPower();
}

QueueHandle_t* Radio_GetTxQueue(void)
{
  return &tx_queue;
}

QueueHandle_t* Radio_GetRxQueue(void)
{
  return &rx_queue;
}

// Radio manager task
static void RadioTask(void* pvParameters)
{
  uint32_t encoded_audio_size;
  uint32_t ax25_size;
  RadioPacket_t packet_out;

  // Airtime / radio management loop
  while (true)
  {
    // Wait to get a packet
    if (!xQueueReceive(tx_queue, &packet_out, portMAX_DELAY))
    {
      continue;
    }

    LOG_I(TAG, "[TX] %.*s", (int)packet_out.frame.PayloadLength, packet_out.payload);

    // We have something to transmit, build and encode the packet
    packet_out.frame.Path = packet_out.path;
    packet_out.frame.Payload = packet_out.payload;
    ax25_size = Ax25BuildUnPacket(&packet_out.frame, ax25_buffer);

    // Audio encoding
    encoded_audio_size = Afsk_HdlcEncode(ax25_buffer,
      ax25_size,
      packet_out.frame.PreFlagCount,
      ax25_size - packet_out.frame.PostFlagCount,
      audio_out,
      AUDIO_BUFFER_SIZE
    );

    // If the encoded audio length is zero, the encode output buffer wasn't big enough to accomidate the entire payload
    // It might be mission critical that this gets out in a worst case scenario
    // So just transmit the entire buffer...
    if (!encoded_audio_size)
    {
      encoded_audio_size = AUDIO_BUFFER_SIZE;
    }

    // Start xmit
    LedOn(LED_2);
    Dra818IoPttOn();
    Dra818IoSetHighRfPower();

    // Wait to play until after we PTT down
    vTaskDelay(PTT_DOWN_DELAY / portTICK_PERIOD_MS);

    // Play audio
    AudioPlay(audio_out, encoded_audio_size);

    // Block while we're transmitting
    AudioOutWait(portMAX_DELAY);

    // Wait a little longer before we PTT up
    vTaskDelay(PTT_UP_DELAY / portTICK_PERIOD_MS);

    // Stop Xmit
    Dra818IoPttOff();
    Dra818IoSetLowRfPower();
    LedOff(LED_2);
  }
}