#include "ColorStack.hpp"

int ColorStackInit(ColorStackNode *(&s))
{
    if ((s = (ColorStackNode *)malloc(sizeof(ColorStackNode))) == NULL)
        return ERROR;
    s->next = NULL;
    s->regionID = -2;
    return OK;
}
int GetColorStackLen(ColorStackNode *s)
{
    int length = 0;
    ColorStackNode *temp = s->next;
    while (temp != NULL)
    {
        length++;
        temp = temp->next;
    }
    return length;
}

int ColorStackPush(ColorStackNode *(&s), int regionID, int color)
{
    ColorStackNode *p;
    if ((p = (ColorStackNode *)malloc(sizeof(ColorStackNode))) == NULL)
        return ERROR;
    p->regionID = regionID;
    p->color = color;
    p->next = s->next;
    s->next = p;
    return OK;
}
int ColorStackPop(ColorStackNode *(&s), int *regionID)
{
    ColorStackNode *p;
    if (s->next != NULL)
    {
        p = s->next;
        *regionID = s->next->regionID;
        s->next = s->next->next;
        free(p);
        p = NULL;
        return OK;
    }
    else
        return ERROR;
}