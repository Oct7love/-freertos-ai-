#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include "APP/bsp_system.h"
#include "stm32f1xx_hal.h"
#define RINGBUFFER_SIZE (1024)

typedef struct {
    volatile uint32_t w;
    volatile uint32_t r;
    uint8_t buffer[RINGBUFFER_SIZE];
}ringbuffer_t;

void ringbuffer_init(ringbuffer_t *rb);
uint8_t ringbuffer_is_full(ringbuffer_t *rb);
uint8_t ringbuffer_is_empty(ringbuffer_t *rb);
int8_t ringbuffer_write(ringbuffer_t *rb, uint8_t *data, uint32_t num);
int8_t ringbuffer_read(ringbuffer_t *rb, uint8_t *data, uint32_t num);

#endif
