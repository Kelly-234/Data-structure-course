#pragma once

#include "Defs.hpp"
#include "ColorStack.hpp"
#include "AdjacentTable.hpp"
#include "Heap.hpp"
#include "Queue.hpp"

int GetFourColorsImg(AdjacentTable &allim, Queue tinting_queue, Mat maskImage, Mat &hlightd);
int BinarySearch(HeapNode *heap, int kcount, Mat maskImage, Mat hlightd, AdjacentTable allim, double t0);
void ShowHighLightedImg(HeapNode *heap, unsigned int lowerIndex, unsigned int upperIndex, Mat maskImage, Mat hlightd, AdjacentTable allim);