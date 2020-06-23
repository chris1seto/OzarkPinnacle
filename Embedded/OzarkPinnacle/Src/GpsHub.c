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
#include "Log.h"

// Max difference between RTC and GPS time allowed in S
#define MAX_TIME_DELTA      5

// Max GPS packet timeout before we reconfigure in mS
#define MAX_GPS_TIMEOUT      2500
#define GPS_RECONFIGURE_TIMEOUT  5000

#define GPS_EPOCH        1900

// Frequency of situation updates
#define SITUATION_UPDATE_PERIOD    (100 / portTICK_PERIOD_MS)

typedef struct
{
  float lat;
  float lon;
  float altitude;
  float last_altitude;
  float track;
  float last_track;
  float speed;
  float last_speed;
} InternalSituationInfo_t;

static QueueHandle_t situation_queue;

static void GpsHubTask(void* pvParameters);

static const char* TAG = "GPSHUB";

void GpsHub_Init(void)
{
  situation_queue = xQueueCreate(1, sizeof(SituationInfo_t));

  // Create task
  xTaskCreate(GpsHubTask,
    TAG,
    1024,
    NULL,
    6,
    NULL);
}

static void HandleNewZda(const NmeaZda_t* zda)
{
  struct tm time;
  uint32_t gps_time_stamp;
  uint32_t current_time;

  // Ignore if we have no fix
  if (!zda->valid || zda->year == 0)
  {
    return;
  }

  // Get the current time
  current_time = RtcGet();

  // Load the time structure with GPS fields
  time.tm_hour = zda->time.hour;
  time.tm_min  = zda->time.minute;
  time.tm_sec  = zda->time.second;
  time.tm_year = (zda->year - GPS_EPOCH);
  time.tm_mon  = (zda->month - 1);
  time.tm_mday = zda->day;

  // Make timestamp from GPS time
  gps_time_stamp = (uint32_t)mktime(&time);

  // If there is more than 5 second difference between our system time and GPS
  // Because of uint32_t arithmatic, one of these comparsions will always be true if one value is larger than the other
  // If both are true, it means one value is larger than the other, and that difference is more than MAX_TIME_DELTA
  if (current_time - gps_time_stamp >= MAX_TIME_DELTA && gps_time_stamp - current_time >= MAX_TIME_DELTA)
  {
    // Set the new system time
    RtcSet(gps_time_stamp);
    LOG_I(TAG, "RTC set!");
  }
}

bool GpsHub_GetSituation(SituationInfo_t* const situation)
{
  return xQueuePeek(situation_queue, situation, 0);
}

static void GpsHubTask(void* pvParameters)
{
  GenericNmeaMessage_t msg;
  QueueHandle_t* nmea_queue;
  TickType_t last_check_for_timeout_time = 0;
  TickType_t last_gga_time = 0;
  TickType_t last_zda_time = 0;
  TickType_t last_rmc_time = 0;
  TickType_t time_now;

  InternalSituationInfo_t situation = {0};
  SituationInfo_t beacon_info = {0};

  // Get GPS NMEA queue
  nmea_queue = Nmea0183_GetQueue();

  // Something has gone wrong if we never get this
  if (nmea_queue == NULL)
  {
    return;
  }

  // Task loop
  while (true)
  {
    time_now = xTaskGetTickCount();

    // Try to get a new message from NMEA as long as there is data to read
    if (!xQueueIsQueueEmptyFromISR(*nmea_queue))
    {
      // Fetch the message
      if(xQueueReceive(*nmea_queue, &msg, 0))
      {
        // Handle messages
        switch(msg.message_type)
        {
          // Handle GGA message
          case NMEA_MESSAGE_TYPE_GGA:
            // Mark that we did get a packet
            last_gga_time = time_now;

            // Ignore if we have no fix or if the fix is not present
            if (!IsAscii(msg.gga.fix) || msg.gga.fix <= '0')
            {
              continue;
            }

            situation.lat = msg.gga.position.lat;
            situation.lon = msg.gga.position.lon;
            situation.altitude = msg.gga.altitude;
            break;

          // Handle RMC message
          case NMEA_MESSAGE_TYPE_RMC:
            // Mark that we did get a packet
            last_rmc_time = time_now;

            // Ignore if we have no fix
            if (msg.rmc.fix != 'A')
            {
              continue;
            }

            // Exact from the new GPS packet
            situation.last_speed = situation.speed;
            situation.speed = msg.rmc.speed;
            situation.last_track = situation.track;
            situation.track = msg.rmc.track;
            break;

          // Handle ZDA message
          case NMEA_MESSAGE_TYPE_ZDA:
            // Mark that we did get a packet
            last_zda_time = time_now;
            HandleNewZda(&msg.zda);
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

    // Fill out situation info
    beacon_info.lat = situation.lat;
    beacon_info.lon = situation.lon;
    beacon_info.altitude = situation.altitude;
    beacon_info.delta_altitude = situation.altitude - situation.last_altitude;
    beacon_info.speed = situation.speed;
    beacon_info.delta_speed = situation.speed - situation.last_speed;
    beacon_info.track = situation.track;
    beacon_info.delta_track = situation.track - situation.last_track;

    // Enqueue
    xQueueOverwrite(situation_queue, &beacon_info);

    vTaskDelay(SITUATION_UPDATE_PERIOD);
  }
}