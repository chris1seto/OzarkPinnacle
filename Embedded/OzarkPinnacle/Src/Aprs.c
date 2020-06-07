/*
  APRS Packet Encoder
    -- Chris Seto, 2018

  References:
    * http://www.aprs.org/doc/APRS101.PDF

  Example from tracksoar: "/000000h0000.00N/00000.00EO000/000/A=000000/11487/44.37/29.42"

  /000000h0000.00N/00000.00EO000/000/A=000000/11487/44.37/29.42
  /191920h3850.86N/09016.92W-Test123

*/
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include "Aprs.h"

static void MakeTimeHms(uint8_t* buffer, const time_t timestamp);
static void MakePositionCoordinates(uint8_t* buffer, const AprsPositionT* position);

// APRS101, Page 34
// 19 bytes
static void MakePositionCoordinates(uint8_t* buffer, const AprsPositionT* position)
{
  // Get elements of lat
  float lat_abs = fabs(position->Lat);
  float lat_whole = floorf(lat_abs);
  float lat_minutes = (lat_abs - lat_whole) * 60.0;
  float lat_seconds = (lat_minutes - floorf(lat_minutes)) * 100.0;
  uint8_t lat_char = (position->Lat > 0) ? 'N' : 'S';

  // Get elements of lon
  float lon_abs = fabs(position->Lon);
  float lon_whole = floorf(lon_abs);
  float lon_minutes = (lon_abs - lon_whole) * 60.0;
  float lon_seconds = (lon_minutes - floorf(lon_minutes)) * 100.0;
  uint8_t lon_char = (position->Lon > 0) ? 'E' : 'W';

  // Format the buffer
  sprintf((char*)buffer, "%02lu%02lu.%02lu%c%c%03lu%02lu.%02lu%c%c", 
    (uint32_t)lat_whole,
    (uint32_t)lat_minutes,
    (uint32_t)lat_seconds,
    lat_char,
    position->SymbolTable,
    (uint32_t)lon_whole,
    (uint32_t)lon_minutes,
    (uint32_t)lon_seconds,
    lon_char,
    position->Symbol);
}

// APRS101, Page 32
// 7 bytes
static void MakeTimeHms(uint8_t* buffer, const time_t timestamp)
{
  struct tm* time_struct;

  // Extract timestamp
  time_struct = gmtime(&timestamp);

  // Check if we got a valid time back
  if (time_struct == NULL)
  {
    sprintf((char*)buffer, "000000h");
    return;
  }

  // Format buffer
  sprintf((char*)buffer, "%02d%02d%02dh", time_struct->tm_hour, time_struct->tm_min, time_struct->tm_sec);
}

// APRS101, Page 32
const uint32_t Aprs_MakePosition(uint8_t* buffer, const AprsPositionReportT* report)
{
  uint32_t bufferPtr = 0;

  // Position report, with timestamp, no messaging
  buffer[bufferPtr++] = '/';

  // Encode timestamp
  if (!report->Timestamp)
  {
    sprintf((char*)buffer + bufferPtr, "000000h");
  }
  else
  {
    MakeTimeHms(buffer + bufferPtr, (time_t)report->Timestamp);
  }
  bufferPtr += 7;

  // Encode position
  // 19 Bytes
  MakePositionCoordinates(buffer + bufferPtr, &report->Position);
  bufferPtr += 19;

  return bufferPtr;
}

const uint32_t Aprs_MakeExtCourseSpeed(uint8_t* buffer, const uint8_t course, const uint16_t speed)
{
  sprintf((char*)buffer, "%03d/%03d/", course, speed);
  return 8;
}