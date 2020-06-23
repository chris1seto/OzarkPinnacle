#ifndef GPSHUB_H
#define GPSHUB_H

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct
{
  float lat;
  float lon;
  float altitude;
  float delta_altitude;
  float track;
  float delta_track;
  float speed;
  float delta_speed;
} SituationInfo_t;

void GpsHub_Init(void);
bool GpsHub_GetSituation(SituationInfo_t* const situation);

#endif
