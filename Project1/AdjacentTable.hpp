#pragma once

#include "Defs.hpp"
#include "Queue.hpp"

typedef struct AdjacentListNode
{
    int regionID;
    int numAdjacentRegions;
    struct AdjacentListNode *next;
} AdjacentListNode;

typedef struct Region
{
    int regionID;
    Point centerPoint;
    AdjacentListNode *adjacentListHead;
} Region, RegionArray[MAX_RIGION_NUM];

typedef struct AdjacentTable
{
    RegionArray regions;
    int numRegions;
} AdjacentTable;

int ChoosePath(AdjacentTable allim, Queue &tinting_queue);
int IsAllVisited(AdjacentTable allim, const bool *visited);
void ShowAdjacencyList(AdjacentTable allim);
void CaluCenterPoint(AdjacentTable &allim, Mat maskImage, Mat fourColorsImg);              //
int GetNumUnvisitedAdjacentRegion(AdjacentTable allim, int regionID, const bool *visited); //
int GenerateAdjacentTable(AdjacentTable &allim, Mat maskImage, int kcount);                //
int SortAdjacentLists(AdjacentTable &allim);
int GetRegionIDWithMaxUnvisitedAdjacentRegion(AdjacentTable allim, const bool *visited);
