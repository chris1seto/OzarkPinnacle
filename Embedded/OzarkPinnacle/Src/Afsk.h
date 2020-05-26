#ifndef AFSK_H
#define AFSK_H
#include <stdint.h>

uint32_t Afsk_HdlcEncode(const uint8_t* data,
  const uint32_t size,
  const uint32_t start_stuff,
  const uint32_t end_stuff,
  uint8_t* afsk_out,
  const uint32_t afsk_buffer_buffer_size);

#endif
