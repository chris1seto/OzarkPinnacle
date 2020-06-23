#ifndef QUEUEBUFFER_H
#define QUEUEBUFFER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
  uint8_t* buffer;
  uint32_t buffer_size;
  uint32_t count;
  uint32_t head;
  uint32_t tail;
} QueueBuffer_t;

void QueueBuffer_Init(QueueBuffer_t* const q, uint8_t* const buffer, const uint32_t buffer_size);
uint32_t QueueBuffer_Count(const QueueBuffer_t* q);
void QueueBuffer_Append(QueueBuffer_t* const q, const uint8_t x);
bool QueueBuffer_Empty(const QueueBuffer_t* q);
uint8_t QueueBuffer_Get(QueueBuffer_t* const q);
void QueueBuffer_Dequeue(QueueBuffer_t* const q, const uint32_t n);
uint8_t QueueBuffer_Peek(const QueueBuffer_t* q, const uint32_t location);
bool QueueBuffer_AppendBuffer(QueueBuffer_t* const q, const uint8_t* x, const uint32_t append_size);
bool QueueBuffer_PeekBuffer(const QueueBuffer_t* q, const uint32_t location, uint8_t* const buffer, const uint32_t size);

#endif
