#include "PointList.hpp"
#include "Grid.hpp"

int InitPointListNode(PointListNode *&pointListNodePtr)
{
    if ((pointListNodePtr = (PointListNode *)malloc(sizeof(PointListNode))) == NULL)
        return ERROR;
    pointListNodePtr->next = NULL;
    return OK;
}
int AddPointListNode(PointListNode *p, Point newPoint, int sampleIndex)
{
    PointListNode *temp = NULL;
    while (p->next != NULL)
        p = p->next;
    if ((temp = (PointListNode *)malloc(sizeof(PointListNode))) == NULL)
        return ERROR;
    p->next = temp;
    temp->next = NULL;
    temp->point = newPoint;
    temp->sampleIndex = sampleIndex;
    return OK;
}
int DelPointListNode(PointListNode *p, int sampleIndex)
{
    PointListNode *temp = p->next;
    if (p->next == NULL)
        return ERROR;
    while (temp->next != NULL && temp->sampleIndex != sampleIndex)
    {
        p = temp;
        temp = temp->next;
    }
    if (temp->sampleIndex == sampleIndex)
    {
        p->next = temp->next;
        free(temp);
        temp = NULL;
    }
    return OK;
}
int GetNewNearPointListNode(Mat grid, PointListNode randPointNode, Point &newRandNearPoint, int gridRadius, double gridScaleFactor)
{
    int i = 0;
    double randAngle = 0, randRadius;
    RNG rng((unsigned)time(NULL));
    Point nearPoint;
    while (i < GetNewNearPointListNodeIterateLimit)
    {
        randAngle = rng.uniform((double)0, (double)(2 * PI));
        randRadius = (double)gridRadius + 1;
        nearPoint.x = randPointNode.point.x + (int)(randRadius * cos(randAngle));
        nearPoint.y = randPointNode.point.y + (int)(randRadius * sin(randAngle));
        if (nearPoint.x < 0 || nearPoint.x > M || nearPoint.y < 0 || nearPoint.y > N)
            continue;
        if (JudgeNearPoint(nearPoint, gridRadius, gridScaleFactor, grid))
        {
            newRandNearPoint.x = nearPoint.x;
            newRandNearPoint.y = nearPoint.y;
            return OK;
        }
        i++;
    }
    return ERROR;
}
int JudgeNearPoint(Point nearPoint, int gridRadius, double gridScaleFactor, Mat grid)
{
    Mat nearGrid(5, 5, CV_16SC3, Scalar(NONE, NONE, NONE));
    Vec2b posInGrid;
    GetPosInGrid(nearPoint, posInGrid, gridScaleFactor);
    GetNearGrid(grid, posInGrid, nearGrid);
    for (int i = 0; i < nearGrid.rows; i++)
    {
        for (int j = 0; j < nearGrid.cols; j++)
        {
            short *rowInNearGrid = nearGrid.ptr<short>(i);
            if (rowInNearGrid[j * nearGrid.channels()] == NONE && rowInNearGrid[j * nearGrid.channels() + 1] == NONE && rowInNearGrid[j * nearGrid.channels() + 2] == NONE)
                continue;
            double distanceSquared = pow((rowInNearGrid[j * nearGrid.channels()] - nearPoint.x), 2) + pow((rowInNearGrid[j * nearGrid.channels() + 1] - nearPoint.y), 2);
            if (distanceSquared < pow(gridRadius, 2))
                return FALSE;
        }
    }
    return TRUE;
}
int GetPointListLen(PointListNode *p)
{
    int length = 0;
    while (p->next != NULL)
    {
        length++;
        p = p->next;
    }
    return length;
}
void GetRandPointListNode(PointListNode *pointListNodePtr, PointListNode &randPointNode)
{
    RNG rng((unsigned)time(NULL));
    int Rinds = rng.uniform(0, GetPointListLen(pointListNodePtr));
    for (int i = 0; i < Rinds; i++)
        pointListNodePtr = pointListNodePtr->next;
    randPointNode.point = pointListNodePtr->next->point;
    randPointNode.sampleIndex = pointListNodePtr->next->sampleIndex;
}