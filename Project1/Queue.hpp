#pragma once

#include "Defs.hpp"

typedef struct QNode
{
    int data;
    struct QNode *next;
} QNode, *QueuePtr;

typedef struct Queue
{
    QueuePtr front;
    QueuePtr rear;
} Queue;

int QueueInit(Queue &Q);
int QueuePush(Queue &Q, int e);
int QueueIsEmpty(Queue Q);
int QueuePop(Queue &Q, int &e);