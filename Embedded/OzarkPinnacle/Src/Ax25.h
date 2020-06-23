#ifndef AX25_H
#define AX25_H

#include <stdint.h>

#define CALL_SIZE    6

// Ax25 framing struct
typedef struct
{
  uint8_t source[6];
  uint8_t destination[6];
  uint8_t* path;
  uint32_t path_size;
  uint8_t source_ssid;
  uint8_t destination_ssid;
  uint8_t* payload;
  uint32_t payload_size;
  uint8_t pre_flag_count;
  uint8_t post_flag_count;
} Ax25Frame_t;

uint32_t Ax25_BuildUnPacket(const Ax25Frame_t* frame, uint8_t* const output_buffer);

#endif // !AX25_H
