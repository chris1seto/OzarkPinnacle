#ifndef GPSHUB_H
#define GPSHUB_H

#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdint.h>
#include "queue.h"

typedef struct
{
  float Lat;
  float Lon;
  float Altitude;
  float dAltitude;
  float Track;
  float dTrack;
  float Speed;
  float dSpeed;
} SituationInfoT;

void GpsHub_Init(void);
void GpsHub_StartTask(void);
void GpsHub_GetSituation(SituationInfoT* situation);

#endif
