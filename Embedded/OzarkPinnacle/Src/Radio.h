#ifndef RADIO_H
#define RADIO_H

#include "FreeRTOS.h"
#include <stdint.h>
#include "task.h"
#include "queue.h"
#include "Ax25.h"

typedef struct
{
  Ax25Frame_t frame;
  uint8_t path[56];
  uint8_t payload[200];
  TickType_t expiration;
} RadioPacket_t;

void Radio_Init(void);
QueueHandle_t* Radio_GetTxQueue(void);
QueueHandle_t* Radio_GetRxQueue(void);

#endif
