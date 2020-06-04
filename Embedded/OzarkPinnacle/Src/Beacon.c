#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stm32f4xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Ax25.h"
#include "Aprs.h"
#include "Beacon.h"
#include "Led.h"
#include "Rtc.h"
#include "time.h"
#include "Radio.h"
#include "Config.h"
#include "FlashConfig.h"
#include "Bme280Shim.h"
#include "Bsp.h"
#include "GpsHub.h"

// APRS packet buffer
#define PACKET_BUFFER_SIZE    500
static uint8_t aprs_buffer[PACKET_BUFFER_SIZE];

static void BeaconTask(void* pvParameters);
static TaskHandle_t beaconTaskHandle = NULL;

#define FIRST_BEACON_OFFSET    500

#define PREFLAG_COUNT    10
#define POSTFLAG_COUNT    25

void Beacon_Init(void)
{
}

void Beacon_StartTask(void)
{
  // Create task
  xTaskCreate(BeaconTask,
    "Beacon",
    1024,
    NULL,
    3,
    &beaconTaskHandle);
}

#define METERS_TO_FEET  3.28084f
static float Meters2Feet(const float m)
{
  return m * METERS_TO_FEET;
}

static uint32_t CalculateSmartBeacon(const uint32_t minTime,
  const uint32_t maxTime,
  const float speed,
  const float dSpeed,
  const float dHeading,
  const float cS,
  const float cDs,
  const float cDh)
{
  return minTime;
}

static void BeaconTask(void* pvParameters)
{
  QueueHandle_t* tx_queue;
  TickType_t last_task_time = 0;
  uint32_t aprs_size;
  AprsPositionReportT aprs_report;
  RadioPacketT beacon_packet;
  uint32_t beacon_period;
  float temperature = 0;
  float pressure = 0;
  float humidity = 0;
  SituationInfo_t situation = {0};

  // Get config
  ConfigT* config = FlashConfigGetPtr();

  // Get the queues
  tx_queue = RadioGetTxQueue();

  // If we didn't get any queues, something is really wrong
  if (tx_queue == NULL)
  {
    return;
  }

  // Beacon quickly on the first start
  beacon_period = FIRST_BEACON_OFFSET;

  // Beacon loop
  while (true)
  {
    // Block until it's time to start
    vTaskDelayUntil(&last_task_time, beacon_period);

    // Get the latest situation
    GpsHub_GetSituation(&situation);

    // Get the current environmental data
    //Bme280ShimGetTph(&temperature, &pressure, &humidity);

    // Compute smart beacon period
    if (config->Aprs.UseSmartBeacon)
    {
      beacon_period = CalculateSmartBeacon(config->Aprs.BeaconPeriod,
        config->Aprs.SmartBeaconMinimumBeaconPeriod,
        situation.speed, situation.delta_speed,
        situation.delta_track,
        config->Aprs.SmartBeaconWeightSpeed,
        config->Aprs.SmartBeaconWeightdSpeed,
        config->Aprs.SmartBeaconWeightdHeading
      );
    }
    else
    {
      beacon_period = config->Aprs.BeaconPeriod;
    }

    // Configure Ax25 frame
    memcpy(beacon_packet.Frame.Source, config->Aprs.Callsign, 6);
    beacon_packet.Frame.SourceSsid = config->Aprs.Ssid;
    memcpy(beacon_packet.Frame.Destination, "APRS  ", 6);
    beacon_packet.Frame.DestinationSsid = 0;
    memcpy(beacon_packet.Path, config->Aprs.Path, 7);
    beacon_packet.Frame.PathLen = 7;

    beacon_packet.Frame.PreFlagCount = PREFLAG_COUNT;
    beacon_packet.Frame.PostFlagCount = POSTFLAG_COUNT;

    // Fill APRS report
    aprs_report.Timestamp = RtcGet();
    aprs_report.Position.Lat = situation.lat;
    aprs_report.Position.Lon = situation.lon;
    aprs_report.Position.Symbol = config->Aprs.Symbol;
    aprs_report.Position.SymbolTable = config->Aprs.SymbolTable;

    // Build APRS report
    aprs_size = 0;
    aprs_size += AprsMakePosition(aprs_buffer, &aprs_report);
    aprs_size += AprsMakeExtCourseSpeed(aprs_buffer + aprs_size, (uint8_t)situation.track, (uint16_t)situation.speed);

    // Append comment for GPS altitude
    sprintf((char*)aprs_buffer + aprs_size, "A=%06i", (int)Meters2Feet(situation.altitude));
    aprs_size += 8;

    // Add telemetry comment
    sprintf((char*)aprs_buffer + aprs_size, "/Pa=%06i/Rh=%02.02f/Ti=%02.02f/V=%02.02f", (int)pressure, humidity, temperature, BspGetVSense());
    aprs_size += 35;

    // Add the APRS report to the packet
    memcpy(beacon_packet.Payload, aprs_buffer, aprs_size);
    beacon_packet.Frame.PayloadLength = aprs_size;

    // Enqueue the packet in the transmit buffer
    xQueueSendToBack(*tx_queue, &beacon_packet, 0);
  }
}