#include "AdjacentTable.hpp"

int ChoosePath(AdjacentTable adjacentTable, Queue &pathQueue)
{
    Queue tempQueue;
    bool visited[MAX_RIGION_NUM] = {FALSE};
    int tempInt, thisRegionID;

    for (int i = 0; i < adjacentTable.numRegions; i++)
    {
        AdjacentListNode *temp = adjacentTable.regions[i].adjacentListHead;
        if (temp == NULL)
            visited[i] = TRUE;
    }
    QueueInit(pathQueue);
    QueueInit(tempQueue);

    while (!IsAllVisited(adjacentTable, visited))
    {
        thisRegionID = GetRegionIDWithMaxUnvisitedAdjacentRegion(adjacentTable, visited);
        if (!visited[thisRegionID])
        {
            visited[thisRegionID] = TRUE;

            QueuePush(pathQueue, thisRegionID + 1);
            QueuePush(tempQueue, thisRegionID);

            while (!QueueIsEmpty(tempQueue))
            {
                QueuePop(tempQueue, tempInt);
                AdjacentListNode *temp = adjacentTable.regions[tempInt].adjacentListHead;
                if (temp == NULL)
                    break;
                while (temp != NULL)
                {
                    if (!visited[(temp->regionID) - 1])
                    {
                        visited[(temp->regionID) - 1] = TRUE;
                        QueuePush(pathQueue, temp->regionID);
                        QueuePush(tempQueue, (temp->regionID) - 1);
                    }
                    temp = temp->next;
                }
            }
        }
    }
    //输出路线
    cout << "\nThe path is:" << endl;
    Queue temp = pathQueue;
    temp.front = temp.front->next;
    while (temp.front != temp.rear)
    {
        cout << temp.front->data << "->";
        temp.front = temp.front->next;
    }
    cout << temp.front->data << endl;
    return OK;
}
int IsAllVisited(AdjacentTable adjacentTable, const bool *visited)
{
    for (int i = 0; i < adjacentTable.numRegions; i++)
    {
        if (visited[i] == FALSE)
            return FALSE;
    }
    return TRUE;
}
void ShowAdjacencyList(AdjacentTable adjacentTable)
{
    cout << "[Adjacent List]" << endl;
    for (int i = 0; i < adjacentTable.numRegions; i++)
    {
        AdjacentListNode *temp = adjacentTable.regions[i].adjacentListHead;
        if (temp == NULL)
            continue;
        cout << adjacentTable.regions[i].regionID << "-";
        while (temp != NULL && temp->next != NULL)
        {
            cout << temp->regionID << "-";
            temp = temp->next;
        }
        if (temp != NULL && temp->next == NULL)
            cout << temp->regionID << endl;
    }
}
void CaluCenterPoint(AdjacentTable &adjacentTable, Mat imgMask, Mat fourColorsImg)
{
    for (int i = 0; i < adjacentTable.numRegions; i++)
    {
        int centerX = 0, centerY = 0, numPoints = 0;
        for (int j = 1; j < imgMask.rows - 1; j++)
        {
            int *rowInImgMask = imgMask.ptr<int>(j);
            for (int k = 1; k < imgMask.cols - 1; k++)
            {
                if (rowInImgMask[k] == i + 1)
                {
                    centerX += k;
                    centerY += j;
                    numPoints++;
                }
            }
        }
        if (!numPoints)
            numPoints = 1;
        adjacentTable.regions[i].centerPoint.x = centerX / numPoints;
        adjacentTable.regions[i].centerPoint.y = centerY / numPoints;
    }
    for (int i = 0; i < adjacentTable.numRegions; i++)
    {
        char str[10];
        sprintf_s(str, "%d", i + 1);
        putText(fourColorsImg, str, adjacentTable.regions[i].centerPoint, FONT_HERSHEY_PLAIN, 0.8, Scalar(255, 255, 255));
    }
}
int GetNumUnvisitedAdjacentRegion(AdjacentTable adjacentTable, int regionID, const bool *visited)
{
    int length = 0;
    if (visited[adjacentTable.regions[regionID - 1].regionID] == FALSE)
        length++;
    AdjacentListNode *temp = adjacentTable.regions[regionID - 1].adjacentListHead;
    if (temp == NULL)
        return length;
    while (temp != NULL)
    {
        if (visited[temp->regionID - 1] == FALSE)
            length++;
        temp = temp->next;
    }
    return length;
}
//分配各区域颜色？
int GetRegionIDWithMaxUnvisitedAdjacentRegion(AdjacentTable adjacentTable, const bool *visited)
{
    int maxRegionID = NONE, maxNumUnvisited = 0, numUnvisited;
    for (int i = 0; i < adjacentTable.numRegions; i++)
    {
        if (visited[i] == FALSE)
        {
            numUnvisited = GetNumUnvisitedAdjacentRegion(adjacentTable, i + 1, visited);
            if (numUnvisited > maxNumUnvisited)
            {
                maxRegionID = i;
                maxNumUnvisited = numUnvisited;
            }
        }
    }
    if (maxRegionID == NONE)
        return ERROR;
    else
        return maxRegionID;
}
int GenerateAdjacentTable(AdjacentTable &adjacentTable, Mat imgMask, int numSampledPoints)
{
    int regionID1, regionID2, edge[8], ref;
    adjacentTable.numRegions = numSampledPoints;

    for (int i = 0; i < numSampledPoints; i++) //分配区域编号
    {
        adjacentTable.regions[i].regionID = i + 1;
        adjacentTable.regions[i].adjacentListHead = NULL;
    }
    for (int i = 1; i < imgMask.rows - 1; i++)
    {
        int *rowInImgMask1 = imgMask.ptr<int>(i - 1);
        int *rowInImgMask2 = imgMask.ptr<int>(i);
        int *rowInImgMask3 = imgMask.ptr<int>(i + 1);
        for (int j = 1; j < imgMask.cols - 1; j++)
        {
            ref = rowInImgMask2[j];
            edge[0] = rowInImgMask1[j - 1];
            edge[1] = rowInImgMask1[j + 1];
            edge[2] = rowInImgMask1[j + 1];
            edge[3] = rowInImgMask2[j - 1];
            edge[4] = rowInImgMask2[j + 1];
            edge[5] = rowInImgMask3[j - 1];
            edge[6] = rowInImgMask3[j];
            edge[7] = rowInImgMask3[j + 1];
            if (ref == -1)
            {
                for (int k = 0; k < 8; k++)
                {
                    if (edge[k] > 0)
                    {
                        regionID1 = edge[k];
                        break;
                    }
                }
                for (int k = 0; k < 8; k++)
                {
                    if (edge[k] > 0 && edge[k] != regionID1)
                    {
                        regionID2 = edge[k];
                        break;
                    }
                }
                if (adjacentTable.regions[regionID1 - 1].adjacentListHead == NULL)
                {
                    AdjacentListNode *tempNode = NULL;
                    if ((tempNode = (AdjacentListNode *)malloc(sizeof(AdjacentListNode))) == NULL)
                    {
                        cout << "Error: insufficient memory..." << endl;
                        exit(1);
                    }
                    tempNode->next = NULL;
                    tempNode->regionID = regionID2;
                    adjacentTable.regions[regionID1 - 1].adjacentListHead = tempNode;
                }
                else
                {
                    int isDuplicate = FALSE;
                    AdjacentListNode *temp = adjacentTable.regions[regionID1 - 1].adjacentListHead;
                    while (temp != NULL)
                    {
                        if (temp->regionID != regionID2 && temp->next != NULL)
                            temp = temp->next;
                        else if (temp->regionID == regionID2 || regionID2 == adjacentTable.regions[regionID1 - 1].regionID)
                        {
                            isDuplicate = TRUE;
                            break;
                        }
                        else
                            break;
                    }
                    if (!isDuplicate)
                    {
                        AdjacentListNode *tempNode = NULL;
                        if ((tempNode = (AdjacentListNode *)malloc(sizeof(AdjacentListNode))) == NULL)
                        {
                            cout << "Error: insufficient memory..." << endl;
                            exit(1);
                        }
                        tempNode->regionID = regionID2;
                        tempNode->next = NULL;
                        temp->next = tempNode;
                    }
                }
                if (adjacentTable.regions[regionID2 - 1].adjacentListHead == NULL)
                {
                    AdjacentListNode *tempNode = NULL;
                    if ((tempNode = (AdjacentListNode *)malloc(sizeof(AdjacentListNode))) == NULL)
                    {
                        cout << "Error: insufficient memory..." << endl;
                        exit(1);
                    }
                    tempNode->next = NULL;
                    tempNode->regionID = regionID1;
                    adjacentTable.regions[regionID2 - 1].adjacentListHead = tempNode;
                }
                else
                {
                    int isDuplicate = FALSE;
                    AdjacentListNode *temp = adjacentTable.regions[regionID2 - 1].adjacentListHead;
                    while (temp != NULL)
                    {
                        if (temp->regionID != regionID1 && temp->next != NULL)
                            temp = temp->next;
                        else if (temp->regionID == regionID1 || regionID1 == adjacentTable.regions[regionID2 - 1].regionID)
                        {
                            isDuplicate = TRUE;
                            break;
                        }
                        else
                            break;
                    }
                    if (!isDuplicate)
                    {
                        AdjacentListNode *tempNode = NULL;
                        if ((tempNode = (AdjacentListNode *)malloc(sizeof(AdjacentListNode))) == NULL)
                        {
                            cout << "Error: insufficient memory..." << endl;
                            exit(1);
                        }
                        tempNode->regionID = regionID1;
                        tempNode->next = NULL;
                        temp->next = tempNode;
                    }
                }
            }
        }
    }
    SortAdjacentLists(adjacentTable);
    return OK;
}
int SortAdjacentLists(AdjacentTable &adjacentTable)
{
    int length = 1;
    int adjacentListsLen[MAX_RIGION_NUM] = {0};
    for (int i = 0; i < adjacentTable.numRegions; i++)
    {
        AdjacentListNode *temp = adjacentTable.regions[i].adjacentListHead;
        while (temp != NULL)
        {
            length++;
            temp = temp->next;
        }
        adjacentListsLen[i] = length;
        length = 1;
    }
    for (int i = 0; i < adjacentTable.numRegions; i++)
    {
        AdjacentListNode *temp = adjacentTable.regions[i].adjacentListHead;
        while (temp != NULL)
        {
            temp->numAdjacentRegions = adjacentListsLen[temp->regionID - 1];
            temp = temp->next;
        }
    }
    for (int i = 0; i < adjacentTable.numRegions; i++)
    {
        for (int j = 0; j < adjacentListsLen[i] - 2; j++)
        {
            AdjacentListNode *temp = adjacentTable.regions[i].adjacentListHead;
            AdjacentListNode *shadow = NULL;

            for (int k = 0; k < adjacentListsLen[i] - j - 2; k++)
            {
                if (temp->numAdjacentRegions < temp->next->numAdjacentRegions)
                {
                    if (adjacentTable.regions[i].adjacentListHead == temp)
                    {
                        AdjacentListNode *current = temp->next;
                        temp->next = temp->next->next;
                        current->next = temp;
                        adjacentTable.regions[i].adjacentListHead = current;
                        shadow = current;
                    }
                    else
                    {
                        AdjacentListNode *current = temp->next;
                        temp->next = temp->next->next;
                        current->next = temp;
                        shadow->next = current;
                        shadow = current;
                    }
                }
                else
                {
                    shadow = temp;
                    temp = temp->next;
                }
            }
        }
    }
    return OK;
}