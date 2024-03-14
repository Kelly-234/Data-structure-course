#include "Grid.hpp"

int GridInit(Mat &grid, double gridScaleFactor, int &numSampledPoints, PointListNode *pointListNodePtr, int gridRadius)
{
    Point originPoint(ORIGINPOINT);
    Vec2b posInGrid;
    GetPosInGrid(originPoint, posInGrid, gridScaleFactor);
    short *rowInGrid = grid.ptr<short>(posInGrid[1]);
    rowInGrid[posInGrid[0] * grid.channels()] = originPoint.x;
    rowInGrid[posInGrid[0] * grid.channels() + 1] = originPoint.y;
    rowInGrid[posInGrid[0] * grid.channels() + 2] = 0;
    numSampledPoints += 1;
    AddPointListNode(pointListNodePtr, originPoint, 0);
    return OK;
}
void GetPosInGrid(Point point, Vec2b &posInGrid, double gridScaleFactor)
{
    posInGrid[0] = (int)(point.x / gridScaleFactor);
    posInGrid[1] = (int)(point.y / gridScaleFactor);
}
int GetNearGrid(Mat grid, Vec2b posInGrid, Mat &nearGrid)
{
    int j = 0;
    int offsetInGrid[25][2]{
        {-2, -2},
        {-1, -2},
        {0, -2},
        {1, -2},
        {2, -2},
        {-2, -1},
        {-1, -1},
        {0, -1},
        {1, -1},
        {2, -1},
        {-2, 0},
        {-1, 0},
        {0, 0},
        {1, 0},
        {2, 0},
        {-2, 1},
        {-1, 1},
        {0, 1},
        {1, 1},
        {2, 1},
        {-2, 2},
        {-1, 2},
        {0, 2},
        {1, 2},
        {2, 2}};
    for (int i = 0; i < 25; i++)
    {
        if ((posInGrid[0] + offsetInGrid[i][0]) > -3 && (posInGrid[0] + offsetInGrid[i][0]) < grid.cols * grid.channels() && (posInGrid[1] + offsetInGrid[i][1] >= 0 && (posInGrid[1] + offsetInGrid[i][1]) < grid.rows))
        {
            short *rowInGrid = grid.ptr<short>(posInGrid[1] + offsetInGrid[i][1]);
            if (i % 5 == 0 && i != 0)
                j++;
            short *rowInNearGrid = nearGrid.ptr<short>(j);
            rowInNearGrid[(i % 5) * nearGrid.channels()] = rowInGrid[(posInGrid[0] + offsetInGrid[i][0]) * grid.channels()];
            rowInNearGrid[(i % 5) * nearGrid.channels() + 1] = rowInGrid[(posInGrid[0] + offsetInGrid[i][0]) * grid.channels() + 1];
            rowInNearGrid[(i % 5) * nearGrid.channels() + 2] = rowInGrid[(posInGrid[0] + offsetInGrid[i][0]) * grid.channels() + 2];
        }
        else
            continue;
    }
    return OK;
}

void AddPoint2Grid(Mat &grid, Point newRandNearPoint, Vec2b posInGrid, int sampleIndex)
{
    short *rowInGrid = grid.ptr<short>(posInGrid[1]);
    rowInGrid[posInGrid[0] * grid.channels()] = newRandNearPoint.x;
    rowInGrid[posInGrid[0] * grid.channels() + 1] = newRandNearPoint.y;
    rowInGrid[posInGrid[0] * grid.channels() + 2] = sampleIndex;
}