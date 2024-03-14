#pragma once

#include "Defs.hpp"
#include "PointList.hpp"

int GridInit(Mat &Wgarray, double stepk, int &samnumsjud, PointListNode *acts, int r);
void GetPosInGrid(Point pointer1, Vec2b &ceposi, double stepk);
int GetNearGrid(Mat Wgarray, Vec2b ceposi, Mat &nearGrid);
void AddPoint2Grid(Mat &Wgarray, Point result_pt, Vec2b ceposi, int sampleIndex);