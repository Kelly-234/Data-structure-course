#pragma once

#include "Defs.hpp"

typedef struct ColorStackNode
{
    int regionID;
    int color;
    struct ColorStackNode *next;
} ColorStackNode;

int ColorStackInit(ColorStackNode *(&s));
int GetColorStackLen(ColorStackNode *s);
int ColorStackPush(ColorStackNode *(&s), int regionID, int color);
int ColorStackPop(ColorStackNode *(&s), int *regionID);