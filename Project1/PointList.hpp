#pragma once

#include "Defs.hpp"

typedef struct PointListNode
{
    Point point;
    int sampleIndex;
    struct PointListNode *next;
} PointListNode;

int InitPointListNode(PointListNode *&acts);
int AddPointListNode(PointListNode *p, Point pointer1, int sampleIndex);
int DelPointListNode(PointListNode *p, int sampleIndex);
int GetNewNearPointListNode(Mat Wgarray, PointListNode ref_pt, Point &result_pt, int r, double stepk);
int JudgeNearPoint(Point pointer1, int r, double stepk, Mat Wgarray);
int GetPointListLen(PointListNode *p);
void GetRandPointListNode(PointListNode *acts, PointListNode &ref_pt);