// 顺序循环队列
// 方案1：牺牲一个单元来区分队空和队满
// 队尾指针指向 队尾元素的后一个位置（下一个应该插入的位置）
#include "queue.h"
#include "stdbool.h"

#include "queue.h"
#include "stdbool.h"

// 初始化队列
void Init_Queue(SeqQueue_t *q) {
  if (q == NULL)
    return;
  q->front = q->rear = 0;
}

// 判断队列是否为空
bool Queue_Empty(SeqQueue_t *q) {
  if (q == NULL)
    return true;
  return (q->rear == q->front);
}

// 判断队列是否满
bool Queue_Full(SeqQueue_t *q) {
  if (q == NULL)
    return false;
  return ((q->rear + 1) % MAX_SIZE == q->front);
}

// 入队
bool EnQueue(SeqQueue_t *q, ElemType x) {
  if (q == NULL || Queue_Full(q))
    return false;

  q->data[q->rear] = x;
  q->rear = (q->rear + 1) % MAX_SIZE;
  return true;
}

// 出队
bool DeQueue(SeqQueue_t *q, ElemType *x) {
  if (q == NULL || x == NULL || Queue_Empty(q))
    return false;

  *x = q->data[q->front];
  q->front = (q->front + 1) % MAX_SIZE;
  return true;
}

// 获取队头元素
bool GetHead(SeqQueue_t *q, ElemType *x) {
  if (q == NULL || x == NULL || Queue_Empty(q))
    return false;

  *x = q->data[q->front];
  return true;
}

// 队列元素个数
uint16_t QueueNum(SeqQueue_t *q) {
  if (q == NULL)
    return 0;
  return (q->rear - q->front + MAX_SIZE) % MAX_SIZE;
}
