#include "Queue.hpp"

int QueueInit(Queue &Q)
{
    Q.front = Q.rear = (QueuePtr)malloc(sizeof(QNode));
    if (!Q.front)
        return ERROR;
    Q.front->next = NULL;
    return OK;
}
int QueuePush(Queue &Q, int e)
{
    QueuePtr s;
    if ((s = (QueuePtr)malloc(sizeof(QNode))) == NULL)
        return ERROR;
    s->data = e;
    s->next = NULL;
    Q.rear->next = s;
    Q.rear = s;
    return OK;
}
int QueueIsEmpty(Queue Q)
{
    if (Q.front->next == NULL)
        return TRUE;
    else
        return FALSE;
}
int QueuePop(Queue &Q, int &e)
{
    QueuePtr p;
    if (Q.front == Q.rear)
        return ERROR;

    p = Q.front->next;
    e = p->data;
    Q.front->next = p->next;
    if (p == Q.rear)
        Q.rear = Q.front;
    free(p);
    p = NULL;
    return OK;
}