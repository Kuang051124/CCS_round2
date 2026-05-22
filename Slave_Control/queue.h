#include "ti_msp_dl_config.h"

#ifndef _QUEUE_H_
#define _QUEUE_H_

#define MAX_SIZE 100 // 定义队列中元素的最大个数
typedef struct {
  uint16_t x;
  uint16_t y;
} Point_t;

typedef Point_t ElemType; // 假设元素类型为int

typedef struct {
  ElemType data[MAX_SIZE]; // 存放队列元素
  uint16_t front;          // 队头指针
  uint16_t rear;           // 队尾指针
} SeqQueue_t;
// 不可能超过 4096 个吧

// 初始化队列
void Init_Queue(SeqQueue_t *q);

// 判断队列是否为空
bool Queue_Empty(SeqQueue_t *q);

// 判断队列是否满
bool Queue_Full(SeqQueue_t *q);

// 入队
bool EnQueue(SeqQueue_t *q, ElemType x);

// 出队
bool DeQueue(SeqQueue_t *q, ElemType *x);

// 获取队头元素
bool GetHead(SeqQueue_t *q, ElemType *x);

// 队列元素个数
uint16_t QueueNum(SeqQueue_t *q);

#endif // _QUEUE_H_