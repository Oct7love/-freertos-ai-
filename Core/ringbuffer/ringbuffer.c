#include "ringbuffer.h"

// 初始化环形缓冲区
void ringbuffer_init(ringbuffer_t *rb)
{
    rb->r = 0;
    rb->w = 0;
    memset(rb->buffer, 0, sizeof(uint8_t) * RINGBUFFER_SIZE);
}

// 检查环形缓冲区是否已满（无锁算法：保留1个空位区分满/空）
uint8_t ringbuffer_is_full(ringbuffer_t *rb)
{
    return ((rb->w + 1) % RINGBUFFER_SIZE == rb->r);
}

// 检查环形缓冲区是否为空（无锁算法：读写指针相等表示空）
uint8_t ringbuffer_is_empty(ringbuffer_t *rb)
{
    return (rb->w == rb->r);
}

// 向环形缓冲区写入数据（无锁算法：只修改写指针，ISR安全）
int8_t ringbuffer_write(ringbuffer_t *rb, uint8_t *data, uint32_t num)
{
    if (num == 0)
        return 0;

    // 计算剩余空间（保留1个空位）
    uint32_t free = (rb->r - rb->w - 1 + RINGBUFFER_SIZE) % RINGBUFFER_SIZE;

    if (free == 0)
        return -1;  // 缓冲区已满

    // 限制写入长度不超过剩余空间
    if (num > free)
        num = free;

    // 循环写入数据
    while (num--)
    {
        rb->buffer[rb->w] = *data++;
        rb->w = (rb->w + 1) % RINGBUFFER_SIZE;
        // 不再修改 itemCount，避免竞态条件
    }

    return 0;
}

// 从环形缓冲区读取数据（无锁算法：只修改读指针，任务安全）
int8_t ringbuffer_read(ringbuffer_t *rb, uint8_t *data, uint32_t num)
{
    if (num == 0)
        return 0;

    // 计算当前数据量
    uint32_t count = (rb->w - rb->r + RINGBUFFER_SIZE) % RINGBUFFER_SIZE;

    if (count == 0)
        return -1;  // 缓冲区为空

    // 限制读取长度不超过当前数据量
    if (num > count)
        num = count;

    // 循环读取数据
    while (num--)
    {
        *data++ = rb->buffer[rb->r];
        rb->r = (rb->r + 1) % RINGBUFFER_SIZE;
        // 不再修改 itemCount，避免竞态条件
    }

    return 0;
}