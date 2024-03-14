#include "Algorithm.hpp"

int GetFourColorsImg(AdjacentTable &adjacentTable, Queue pathQueue, Mat imgMask, Mat &highlightedImg)
{
    int popRecords[MAX_RIGION_NUM] = {0};
    bool colorUsedFlags[MAX_RIGION_NUM][4] = {0};
    Queue bufferQueue;
    Queue transferStationQueue;
    QueueInit(bufferQueue);
    QueueInit(transferStationQueue);
    ColorStackNode *colorStack = NULL;
    ColorStackInit(colorStack);

    int thisRegionID;
    while (!QueueIsEmpty(pathQueue) || !QueueIsEmpty(bufferQueue))
    {
        for (int i = 0; i < adjacentTable.numRegions; i++)
        {
            if (popRecords[i] >= POP_RECORDS_LIMIT)
            {
                popRecords[i] = 0;
                for (int j = 0; j < 20; j++)
                {
                    int temp_area_number;
                    ColorStackPop(colorStack, &temp_area_number);
                    QueuePush(transferStationQueue, temp_area_number);
                    while (!QueueIsEmpty(bufferQueue))
                    {
                        int transfer;
                        QueuePop(bufferQueue, transfer);
                        QueuePush(transferStationQueue, transfer);
                    }
                    while (!QueueIsEmpty(transferStationQueue))
                    {
                        int transfer;
                        QueuePop(transferStationQueue, transfer);
                        QueuePush(bufferQueue, transfer);
                    }
                }
            }
        }
        if (!QueueIsEmpty(bufferQueue))
        {
            QueuePop(bufferQueue, thisRegionID);
            bool left_colors_table[4] = {0};
            if (GetColorStackLen(colorStack))
            {
                ColorStackNode *search_stack = colorStack->next;
                while (search_stack != NULL)
                {
                    AdjacentListNode *temp = adjacentTable.regions[thisRegionID - 1].adjacentListHead;
                    while (temp != NULL)
                    {
                        if (temp->regionID == search_stack->regionID)
                            left_colors_table[search_stack->color] = 1;
                        temp = temp->next;
                    }
                    search_stack = search_stack->next;
                }
                int tinted = FALSE;
                for (int i = 0; i < 4; i++)
                {
                    if (left_colors_table[i] != 1 && colorUsedFlags[thisRegionID - 1][i] == 0)
                    {
                        ColorStackPush(colorStack, thisRegionID, i);
                        colorUsedFlags[thisRegionID - 1][i] = 1;
                        tinted = TRUE;
                        break;
                    }
                }
                if (tinted == FALSE)
                {
                    for (int i = 0; i < 4; i++)
                        colorUsedFlags[thisRegionID - 1][i] = 0;

                    int temp_area_number;
                    ColorStackPop(colorStack, &temp_area_number);
                    popRecords[temp_area_number - 1]++;
                    QueuePush(transferStationQueue, temp_area_number);
                    QueuePush(transferStationQueue, thisRegionID);
                    while (!QueueIsEmpty(bufferQueue))
                    {
                        int transfer;
                        QueuePop(bufferQueue, transfer);
                        QueuePush(transferStationQueue, transfer);
                    }
                    while (!QueueIsEmpty(transferStationQueue))
                    {
                        int transfer;
                        QueuePop(transferStationQueue, transfer);
                        QueuePush(bufferQueue, transfer);
                    }
                }
            }
            else
            {
                int flag = FALSE;
                for (int i = 0; i < 4; i++)
                {
                    if (colorUsedFlags[thisRegionID - 1][i] != 1)
                    {
                        ColorStackPush(colorStack, thisRegionID, i);
                        colorUsedFlags[thisRegionID - 1][i] = 1;
                        flag = TRUE;
                        break;
                    }
                }
                if (flag == FALSE)
                    cout << "Coloring failure!" << endl;
            }
        }
        else if (QueueIsEmpty(bufferQueue))
        {
            QueuePop(pathQueue, thisRegionID);
            bool left_colors_table[4] = {0};
            if (GetColorStackLen(colorStack))
            {
                ColorStackNode *search_stack = colorStack->next;
                while (search_stack != NULL)
                {
                    AdjacentListNode *temp = adjacentTable.regions[thisRegionID - 1].adjacentListHead;
                    while (temp != NULL)
                    {
                        if (temp->regionID == search_stack->regionID)
                            left_colors_table[search_stack->color] = 1;
                        temp = temp->next;
                    }
                    search_stack = search_stack->next;
                }
                int tinted = FALSE;
                for (int i = 0; i < 4; i++)
                {
                    if (left_colors_table[i] != 1)
                    {
                        ColorStackPush(colorStack, thisRegionID, i);
                        colorUsedFlags[thisRegionID - 1][i] = 1;
                        tinted = TRUE;
                        break;
                    }
                }
                if (tinted == FALSE)
                {
                    int temp_area_number;
                    ColorStackPop(colorStack, &temp_area_number);
                    popRecords[temp_area_number]++;
                    QueuePush(bufferQueue, temp_area_number);
                    QueuePush(bufferQueue, thisRegionID);
                }
            }
            else
            {
                ColorStackPush(colorStack, thisRegionID, RED);
                colorUsedFlags[thisRegionID - 1][RED] = 1;
            }
        }
    }

    Mat img3C = imread(IMGPATH), grayImg3C;
    cvtColor(img3C, grayImg3C, COLOR_BGR2GRAY);
    cvtColor(grayImg3C, grayImg3C, COLOR_GRAY2BGR);
    Mat fourColorsImg(imgMask.size(), CV_8UC3);

    for (int i = 0; i < imgMask.rows; i++)
        for (int j = 0; j < imgMask.cols; j++)
        {
            int regionID = imgMask.at<int>(i, j);
            ColorStackNode *temp = colorStack->next;
            if (regionID > 0)
            {
                while (temp != NULL)
                {
                    if (temp->regionID == regionID)
                    {
                        switch (temp->color)
                        {
                        case RED:
                            fourColorsImg.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
                            highlightedImg.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
                            break;
                        case GREEN:
                            fourColorsImg.at<Vec3b>(i, j) = Vec3b(0, 255, 0);
                            highlightedImg.at<Vec3b>(i, j) = Vec3b(0, 255, 0);
                            break;
                        case BLUE:
                            fourColorsImg.at<Vec3b>(i, j) = Vec3b(255, 0, 0);
                            highlightedImg.at<Vec3b>(i, j) = Vec3b(255, 0, 0);
                            break;
                        case YELLOW:
                            fourColorsImg.at<Vec3b>(i, j) = Vec3b(0, 255, 255);
                            highlightedImg.at<Vec3b>(i, j) = Vec3b(0, 255, 255);
                            break;
                        }
                    }
                    temp = temp->next;
                }
            }
            else if (regionID == -1)
            {
                fourColorsImg.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
                highlightedImg.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
            }
        }

    fourColorsImg = fourColorsImg * 0.5 + grayImg3C * 0.5;
    CaluCenterPoint(adjacentTable, imgMask, fourColorsImg);
    imshow("[four primary colour]", fourColorsImg);
    return OK;
}

void ShowHighLightedImg(HeapNode *heap, unsigned int lowerIndex, unsigned int upperIndex, Mat imgMask, Mat highlightedImg, AdjacentTable adjacentTable)
{
    Mat img3C = imread(IMGPATH), grayImg3C, highlightedImgCopy;
    cvtColor(img3C, grayImg3C, COLOR_BGR2GRAY);
    cvtColor(grayImg3C, grayImg3C, COLOR_GRAY2BGR);
    highlightedImg.copyTo(highlightedImgCopy);

    int index, is_target = FALSE;
    char str[10];

    //查询区域面积是否在范围内，在则涂白，不在则涂黑（gaoguang1_copy）
    for (int i = 0; i < imgMask.rows; i++)
        for (int j = 0; j < imgMask.cols; j++)
        {
            index = imgMask.at<int>(i, j);
            if (index > 0)
            {
                is_target = FALSE;
                for (int k = lowerIndex; k <= upperIndex; k++)
                {
                    if (index == heap[k].id)
                    {
                        is_target = TRUE;
                        break;
                    }
                }
                if (!is_target)
                    highlightedImgCopy.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
            }
            else if (index == -1)
                highlightedImgCopy.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
        }
    highlightedImgCopy = highlightedImgCopy * 0.5 + grayImg3C * 0.5;

    //输出区域编号
    for (int i = lowerIndex; i <= upperIndex; i++)
    {
        sprintf_s(str, "%d", heap[i].id);
        putText(highlightedImgCopy, str, adjacentTable.regions[heap[i].id - 1].centerPoint, FONT_HERSHEY_PLAIN, 0.8, Scalar(255, 255, 255));
    }
    imshow("Binary Search", highlightedImgCopy);
}
int BinarySearch(HeapNode *heap, int numAreas, Mat imgMask, Mat highlightedImg, AdjacentTable adjacentTable,double t0)
{
    unsigned int lowerBound, upperBound, lowerIndex = 0, upperIndex = 0;
    unsigned int low = 1, high = numAreas, mid;
    int temp, count = 0;
    //输入
    while (1)
    {
        cout << "the area range:" << heap[1].area << "~" << heap[numAreas].area << endl;
        cout << "lower bound(-1 for quit):";
        cin >> temp;
        string num_input_string;
        while (cin.fail() || (temp < (int)heap[1].area && temp!=-1) || (temp > heap[numAreas].area))
        {
            if (cin.fail())
            {
                cin.clear();
                cin >> num_input_string;
                cout << "Please input a number:";
            }
            else
            {
                if(temp < 0)
                    cout << "Please input a positive number." << endl;
                else if (temp < (int)heap[1].area && temp != -1)
                    cout << "the lower bound must >=" << heap[1].area << "." << endl;
                else
                    cout << "the lower bound must <= " << heap[numAreas].area << "." << endl;
                cout << "lower bound(-1 for quit):";
            }
            cin >> temp;
        }
        if (temp == -1)
            return EXIT;
        lowerBound = (unsigned int)temp;
        cout << "upper bound(-1 for quit):";
        cin >> temp;
        while (cin.fail() || ((temp < (int)lowerBound || temp > (int)heap[numAreas].area) && temp!=-1))
        {
            if (cin.fail())
            {
                cin.clear();
                cin >> num_input_string;
                cout << "Please input a number:";
            }
            else
            {
                cout << "the upper bound must >=" << lowerBound << " and <=" << heap[numAreas].area << "." << endl;
                cout << "upper bound(-1 for quit):";
            }
            cin >> temp;
        }
        if (temp == -1)
            return EXIT;
        upperBound = (unsigned int)temp;
        break;
    }
    //查找下界
    while (low <= high)
    {
        mid = (low + high) / 2;
        if (heap[mid].area == lowerBound)
        {
            while (mid)
            {
                if (heap[mid - 1].area == lowerBound)
                    mid--;
                else
                    break;

            }
            lowerIndex = mid;
            break;
        }
        else if (heap[mid].area < lowerBound)
        {
            low = mid + 1;
            lowerIndex = low;
        }
        else
            high = mid - 1;
    }
    //查找上界
    high = numAreas;
    while (low <= high)
    {
        mid = (low + high) / 2;
        if (heap[mid].area == upperBound)
        {
            while (mid<numAreas-1)
            {
                if (heap[mid + 1].area == upperBound)
                    mid++;
                else
                    break;
            }
            upperIndex = mid;
            break;
        }
        else if (heap[mid].area < upperBound)
            low = mid + 1;
        else
        {
            high = mid - 1;
            upperIndex = high;
        }
    }
    if (lowerIndex > upperIndex)
        cout << "No region area between" << lowerBound << "~" << upperBound << "." << endl;
    else
    {
        count = 0;
        cout << "regions the area between" << lowerBound << "~" << upperBound << "are:" << endl;
        for (unsigned int i = lowerIndex; i <= upperIndex; i++)
        {
            cout << heap[i].id << ": " << heap[i].area << endl;
            count++;
        }
        cout << count << " areas in total." << endl;
        cout << "Elapsed time:" << t0 << endl;
    }

    ShowHighLightedImg(heap, lowerIndex, upperIndex, imgMask, highlightedImg, adjacentTable);

    cout << "\nClick [MISSION 1 RESULT] for search again." << endl;
    waitKey(0);
    return OK;
}