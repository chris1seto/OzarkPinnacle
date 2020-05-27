#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f4xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Nmea0183.h"
#include "Rtc.h"
#include "time.h"
#include "UbloxNeo.h"
#include "GpsHub.h"
#include "Helpers.h"

// Max difference between RTC and GPS time allowed in S
#define MAX_TIME_DELTA      5

// Max GPS packet timeout before we reconfigure in mS
#define MAX_GPS_TIMEOUT      2500
#define GPS_RECONFIGURE_TIMEOUT  5000

#define GPS_EPOCH        1900

// Frequency of situation updates
#define SITUATION_UPDATE    1000

typedef struct
{
  float Lat;
  float Lon;
  float Altitude;
  float LastAltitude;
  float Track;
  float LastTrack;
  float Speed;
  float LastSpeed;
} InternalSituationInfo_t;

static QueueHandle_t situation_queue;

static void GpsHubTask(void* pvParameters);

void GpsHub_Init(void)
{
  situation_queue = xQueueCreate(1, sizeof(SituationInfoT));

  if (situation_queue == NULL)
  {
    return;
  }
}

void GpsHub_StartTask(void)
{
  // Create task
  xTaskCreate(GpsHubTask,
    "GpsHub",
    300,
    NULL,
    5,
    NULL);
}

static void HandleNewZda(const NmeaZdaT* zda)
{
  struct tm time;
  uint32_t gps_time_stamp;
  uint32_t current_time;

  // Ignore if we have no fix
  if (!zda->Valid || zda->Year == 0)
  {
    return;
  }

  // Get the current time
  current_time = RtcGet();

  // Load the time structure with GPS fields
  time.tm_hour = zda->Time.Hour;
  time.tm_min  = zda->Time.Minute;
  time.tm_sec  = zda->Time.Second;
  time.tm_year = (zda->Year - GPS_EPOCH);
  time.tm_mon  = (zda->Month - 1);
  time.tm_mday = zda->Day;

  // Make timestamp from GPS time
  gps_time_stamp = (uint32_t)mktime(&time);

  // If there is more than 5 second difference between our system time and GPS
  // Because of uint32_t arithmatic, one of these comparsions will always be true if one value is larger than the other
  // If both are true, it means one value is larger than the other, and that difference is more than MAX_TIME_DELTA
  if (current_time - gps_time_stamp >= MAX_TIME_DELTA && gps_time_stamp - current_time >= MAX_TIME_DELTA)
  {
    // Set the new system time
    RtcSet(gps_time_stamp);
    printf("Set RTC\r\n");
  }
}

void GpsHub_GetSituation(SituationInfoT* situation)
{
  xQueuePeek(situation_queue, situation, 0);
}

static void GpsHubTask(void* pvParameters)
{
  GenericNmeaMessageT msg;
  QueueHandle_t* nmea_queue;
  TickType_t last_task_time = 0;
  TickType_t last_situation_update_time = 0;

  TickType_t last_check_for_timeout_time = 0;
  TickType_t last_gga_time = 0;
  TickType_t last_zda_time = 0;
  TickType_t last_rmc_time = 0;
  TickType_t time_now;

  InternalSituationInfo_t situation = {0};
  SituationInfoT beacon_info;

  // Get GPS NMEA queue
  nmea_queue = Nmea0183GetQueue();

  // Something has gone wrong if we never get this
  if (nmea_queue == NULL)
  {
    return;
  }

  // Task loop
  while (true)
  {
    // Block until it's time to start
    vTaskDelayUntil(&last_task_time, 100);

    time_now = xTaskGetTickCount();

    // Try to get a new message from NMEA as long as there is data to read
    if(!xQueueIsQueueEmptyFromISR(*nmea_queue))
    {
      // Fetch the message
      if(xQueueReceive(*nmea_queue, &msg, 0))
      {
        // Handle messages
        switch(msg.MessageType)
        {
          // Handle GGA message
          case NMEA_MESSAGE_TYPE_GGA :
            // Mark that we did get a packet
            last_gga_time = time_now;

            // Ignore if we have no fix or if the fix is not present
            if (!IsAscii(msg.Gga.Fix) || msg.Gga.Fix <= '0')
            {
              continue;
            }

            situation.Lat = msg.Gga.Position.Lat;
            situation.Lon = msg.Gga.Position.Lon;
            situation.Altitude = msg.Gga.Altitude;
            break;

          // Handle RMC message
          case NMEA_MESSAGE_TYPE_RMC :
            // Mark that we did get a packet
            last_rmc_time = time_now;

            // Ignore if we have no fix
            if (msg.Rmc.Fix != 'A')
            {
              continue;
            }

            // Exact from the new GPS packet
            situation.LastSpeed = situation.Speed;
            situation.Speed = msg.Rmc.Speed;
            situation.LastTrack = situation.Track;
            situation.Track = msg.Rmc.Track;
            break;

          // Handle ZDA message
          case NMEA_MESSAGE_TYPE_ZDA :
            // Mark that we did get a packet
            last_zda_time = time_now;
            HandleNewZda(&msg.Zda);
            break;

        default:
        case NMEA_MESSAGE_TYPE_GSA:
          break;
        }
      }
    }

    // Check for GPS timeouts on packets
    if (time_now - last_check_for_timeout_time > GPS_RECONFIGURE_TIMEOUT)
    {
      last_check_for_timeout_time = time_now;

      if (time_now - last_gga_time > MAX_GPS_TIMEOUT)
      {
        UbloxNeo_SetOutputRate("GGA", 1);
      }

      if (time_now - last_rmc_time > MAX_GPS_TIMEOUT)
      {
        UbloxNeo_SetOutputRate("RMC", 1);
      }

      if (time_now - last_zda_time > MAX_GPS_TIMEOUT)
      {
        UbloxNeo_SetOutputRate("ZDA", 2);
      }
    }

    // 1hz updates to situation
    if (time_now - last_situation_update_time > SITUATION_UPDATE)
    {
      // Fill out situation info
      beacon_info.Lat = situation.Lat;
      beacon_info.Lon = situation.Lon;
      beacon_info.Altitude = situation.Altitude;
      beacon_info.dAltitude = situation.Altitude - situation.LastAltitude;
      beacon_info.Speed = situation.Speed;
      beacon_info.dSpeed = situation.Speed - situation.LastSpeed;
      beacon_info.Track = situation.Track;
      beacon_info.dTrack = situation.Track - situation.LastTrack;

      // Enqueue
      xQueueOverwrite(situation_queue, &beacon_info);
    }
  }
}