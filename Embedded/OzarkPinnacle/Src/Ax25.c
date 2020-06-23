#include <stdint.h>
#include <string.h>
#include "Ax25.h"
#include "CrcCcitt.h"

#define FLAG_BYTE    0x7e

uint32_t Ax25_BuildUnPacket(const Ax25Frame_t* frame, uint8_t* const output_buffer)
{
  uint32_t i = 0;
  uint32_t output_buffer_ptr = 0;
  uint16_t crc;

  // Begin building the packet buffer

  // Write preamble to output buffer
  for(i = 0 ; i < frame->pre_flag_count ; i++)
  {
    output_buffer[output_buffer_ptr++] = FLAG_BYTE;
  }

  // Destination callsign
  memcpy(output_buffer + output_buffer_ptr, frame->destination, CALL_SIZE);
  output_buffer_ptr += CALL_SIZE;

  // Destination SSID
  output_buffer[output_buffer_ptr++] = 0b01110000 | (frame->destination_ssid & 0x0f);

  // Source callsign
  memcpy(output_buffer + output_buffer_ptr, frame->source, CALL_SIZE);
  output_buffer_ptr += CALL_SIZE;

  // Source callsign ssid
  output_buffer[output_buffer_ptr++] = 0b00110000 | (frame->source_ssid & 0x0f);

  // Path
  memcpy(output_buffer + output_buffer_ptr, frame->path, frame->path_size);
  output_buffer_ptr +=  frame->path_size;

  // For each byte in the addresses, we need to left shift by 1
  // The last address byte's LSB is set to indicate end of address bits
  for (i = frame->pre_flag_count; i < output_buffer_ptr; i++)
  {
    output_buffer[i] <<= 1;
  }

  output_buffer[output_buffer_ptr - 1] |= 0x01;

  // Control field
  output_buffer[output_buffer_ptr++] = 0x03;

  // Packet ID
  output_buffer[output_buffer_ptr++] = 0xF0;

  // Payload
  memcpy(output_buffer + output_buffer_ptr, frame->payload, frame->payload_size);
  output_buffer_ptr += frame->payload_size;

  // Compute the checksum
  crc = CrcCcitt(output_buffer + frame->pre_flag_count, output_buffer_ptr - frame->pre_flag_count);

  output_buffer[output_buffer_ptr++] = crc;
  output_buffer[output_buffer_ptr++] = (crc >> 8);

  // Write postamble to output buffer
  for (i = 0; i < frame->post_flag_count; i++)
  {
    output_buffer[output_buffer_ptr++] = FLAG_BYTE;
  }

  return output_buffer_ptr;
}
