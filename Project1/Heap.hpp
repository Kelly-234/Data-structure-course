#pragma once

#include "Defs.hpp" 

typedef struct HeapNode
{
    unsigned int area;
    int id;
} HeapNode;

int HeapInit(HeapNode *heap);
void CaluHeapArea(Mat maskImage, HeapNode *heap);
void HeapSortAsc(HeapNode *heap, unsigned int heapSortLimit);
void MaximizeHeap(HeapNode *heap, unsigned int heapSortLimit);
void MaximizeHeapOnce(HeapNode *heap, int specific, unsigned int heapSortLimit);