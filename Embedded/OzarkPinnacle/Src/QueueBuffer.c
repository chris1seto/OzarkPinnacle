#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "QueueBuffer.h"

void QueueBuffer_Init(QueueBuffer_t* const q, uint8_t* const buffer, const uint32_t buffer_size)
{
  q->buffer = buffer;
  q->buffer_size = buffer_size;

  q->head = 0;
  q->tail = 0;
  q->count = 0;
}

uint32_t QueueBuffer_Count(const QueueBuffer_t* q)
{
  return q->count;
}

void QueueBuffer_Append(QueueBuffer_t* const q, const uint8_t x)
{
  // Append the data and make the new tail location
  q->buffer[q->tail] = x;
  q->tail = ((q->tail + 1) % q->buffer_size);

  // Check if we can expand the count any more
  if (q->count < q->buffer_size)
  {
    q->count++;
  }
}

bool QueueBuffer_Empty(const QueueBuffer_t* q)
{
  return (q->count == 0);
}

uint8_t QueueBuffer_Get(QueueBuffer_t* const q)
{
  uint8_t x = q->buffer[q->head];
  q->head = ((q->head + 1) % q->buffer_size);
  q->count--;
  return x;
}

void QueueBuffer_Dequeue(QueueBuffer_t* const q, const uint32_t n)
{
  if (n > q->count)
  {
    return;
  }

  q->count -= n;
  q->head = (q->head + n) % q->buffer_size;
}

uint8_t QueueBuffer_Peek(const QueueBuffer_t* q, const uint32_t location)
{
  return q->buffer[(q->head + location) % q->buffer_size];
}

bool QueueBuffer_AppendBuffer(QueueBuffer_t* const q, const uint8_t* x, const uint32_t append_size)
{
  uint32_t buffer_end_size = q->buffer_size - q->tail;

  // Check if we can even put this buffer into the queu
  if (q->count + append_size > q->buffer_size)
  {
    return false;
  }

  // Check if there is enough space on the end of the queue to put the entire buffer
  if (buffer_end_size >= append_size)
  {
    // We can write the entire append buffer in one shot
    memcpy((void*)(q->buffer + q->tail), (void*)x, append_size);
  }
  else
  {
    memcpy((void*)(q->buffer + q->tail), (void*)x, buffer_end_size);
    memcpy((void*)(q->buffer), (void*)(x + buffer_end_size), append_size - buffer_end_size);
  }

  // Append to the tail
  q->tail = ((q->tail + append_size) % q->buffer_size);
  q->count += append_size;

  return true;
}

bool QueueBuffer_PeekBuffer(const QueueBuffer_t* q, const uint32_t location, uint8_t* const buffer, const uint32_t size)
{
  uint32_t copy_start = q->head + location;
  uint32_t copy_1_size;

  // Check to see if this amount of length exists at the location
  if (location + size > q->count)
  {
    return false;
  }

  // Check if we can do this in a single shot
  if (copy_start + size <= q->buffer_size)
  {
    memcpy((void*)buffer, (void*)(q->buffer + copy_start), size);
  }
  else
  {
    // Double shot copy
    copy_1_size = (q->buffer_size - copy_start);
    memcpy((void*)buffer, (void*)(q->buffer + copy_start), copy_1_size);
    memcpy((void*)(buffer + copy_1_size), (void*)(q->buffer), (size - copy_1_size));
  }

  return true;
}