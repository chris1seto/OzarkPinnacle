#include <math.h>
#include <stdint.h>
#include "AfskDefs.h"

static void EncodeTone(const uint8_t symbol, uint8_t* buffer, const uint32_t size, float* phase);

void Afsk_Init(void)
{
}

// Include one clock duration of a mark or space tone
static void EncodeTone(const uint8_t symbol, uint8_t* buffer, const uint32_t size, float* phase)
{
  uint32_t i_tone = 0;

  // Select tone
  float delta_phase = symbol ? PHASE_DELTA_MARK : PHASE_DELTA_SPACE;

  // Encode 1bit of this tone
  for (i_tone = 0 ; i_tone < size ; i_tone++)
  {
    // Add sample to buffer
    *(buffer++) = (uint8_t)(127.0 * (sinf(*phase) + 1.0));

    // Increment phase advance
    *phase += delta_phase;

    if (*phase > PI_2)
    {
      *phase -= PI_2;
    }
  }
}

// Encode AFSK with NZRI and bitstuffing
uint32_t Afsk_HdlcEncode(const uint8_t* data,
  const uint32_t size,
  const uint32_t start_stuff,
  const uint32_t end_stuff,
  uint8_t* afsk_out,
  const uint32_t afsk_buffer_buffer_size)
{
  // Iterators
  uint32_t i_byte = 0;
  uint32_t i_bit = 0;

  // Bitstuffing
  uint8_t one_count = 0;

  // Sin modulation
  float phase = 0;

  // Data
  uint8_t working_word = 0;
  uint8_t current_symbol = 0;

  // Buffer
  uint32_t sample_size = 0;

  // For each byte
  for (i_byte = 0 ; i_byte < size ; i_byte++)
  {
    // Get the working word
    working_word = data[i_byte];

    // Encode bits
    for (i_bit = 0; i_bit < 8; i_bit++)
    {
      // NZRI
      if ((working_word & 0x01) == 0)
      {
        current_symbol = !current_symbol;
        one_count = 0;
      }
      else
      {
        one_count++;
      }

      // Encode the bit if we have room
      if (sample_size + TONE_SAMPLE_DURATION > afsk_buffer_buffer_size)
      {
        return 0;
      }

      EncodeTone(current_symbol, afsk_out + sample_size, TONE_SAMPLE_DURATION, &phase);
      sample_size += TONE_SAMPLE_DURATION;

      // Stuff the bit if need be
      if (i_byte > start_stuff && i_byte < end_stuff)
      {
        if (one_count >= STUFFING_SIZE)
        {
          current_symbol = !current_symbol;

          // Encode the bit if we have room
          if (sample_size + TONE_SAMPLE_DURATION > afsk_buffer_buffer_size)
          {
            return 0;
          }

          EncodeTone(current_symbol, afsk_out + sample_size, TONE_SAMPLE_DURATION, &phase);
          sample_size += TONE_SAMPLE_DURATION;
          one_count = 0;
        }
      }
      else
      {
        one_count = 0;
      }

      // Advance to the next bit
      working_word >>= 1;
    }
  }

  // Cap the DAC with zero value
  if (sample_size < afsk_buffer_buffer_size)
  {
    afsk_out[sample_size++] = 0;
  }

  // Return back the actual size of the encoded buffer
  return sample_size;
}