#ifndef DRA818_H
#define DRA818_H

#include <stdint.h>
#include <stdbool.h>

void Dra818_Init(void);
bool Dra818_Connect(void);
bool Dra818_SetGroup(const float txFreq, const float rxFreq, const char ctcssTx[4], const char ctcssRx[4], const uint8_t squelch);
bool Dra818_SetFilter(const uint8_t predeemphasis, const uint8_t highpass, const uint8_t lowpass);

#endif
