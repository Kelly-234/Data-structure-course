/**
 * @file project.cpp
 * @author Kelly
 * @brief 
 * @version 1.0.0
 * @date 2021-10-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "Main.hpp"

int main()
{
	//初始化图像
	Mat img3C = imread(IMGPATH);				 //三通道原图
	Mat emptyImg1C;								 //单通道空白图
	Mat grayImg3C;								 //三通道灰度图
	Mat img2watershed;							 //三通道图，用于分水岭算法
	Mat imgMask(img3C.size(), CV_32S);			 //分割掩码图
	Mat highlightedImg(imgMask.size(), CV_8UC3); //高亮图
	img3C.copyTo(img2watershed);
	cvtColor(img3C, emptyImg1C, COLOR_BGR2GRAY);
	cvtColor(emptyImg1C, grayImg3C, COLOR_GRAY2BGR);
	emptyImg1C = Scalar::all(0);

	//初始化其他变量
	int numSampledPoints = 0; //实际生成的样本点数
	int numGridCols;		  //取样网格列数
	int numGridRows;		  //取样网格行数
	double numSeeds;		  //理论取样点数
	int gridRadius;			  //取样网格半径
	double gridScaleFactor;	  //取样网格缩放因子

	//MISSION 1：随机打点+分水岭
	cout << "\n[MISSION 1]" << endl;
	while (1)
	{
		cout << "Number of seed points:";
		cin >> numSeeds;
		string num_input_string;
		while (cin.fail())
		{
			cin.clear();
			cin >> num_input_string;
			cout << "Please input a number:";
			cin >> numSeeds;
		}
		if (numSeeds != floor(numSeeds))
			cout << "Input error.(please input an integer)" << endl;
		else if (numSeeds <= 1 || numSeeds > 1200)
			cout << "Input error.(between 1~1200)" << endl;
		else
			break;
	}

	//计算取样网格参数
	gridRadius = (int)sqrt(M * N / numSeeds);
	gridScaleFactor = gridRadius / sqrt(2);
	numGridCols = (int)(M / gridScaleFactor) + 1;
	numGridRows = (int)(N / gridScaleFactor) + 1;
	Mat grid(numGridRows, numGridCols, CV_16SC3, Scalar(NONE, NONE, NONE)); //取样网格

	RNG rng((unsigned)time(NULL)); // C++随机数生成器
	double t1 = (double)getTickCount();

	PointListNode *pointListNodePtr = NULL; //取样参考点链表指针
	if (!InitPointListNode(pointListNodePtr) || !GridInit(grid, gridScaleFactor, numSampledPoints, pointListNodePtr, gridRadius))
	{
		cout << "Error: run out of memory." << endl;
		exit(1);
	}

	//生成随机取样点链表
	while (GetPointListLen(pointListNodePtr))
	{
		PointListNode randPointNode; //随机的取样参考点链表
		Point newRandNearPoint;		 //随机取样点
		GetRandPointListNode(pointListNodePtr, randPointNode);
		if (GetNewNearPointListNode(grid, randPointNode, newRandNearPoint, gridRadius, gridScaleFactor))
		{
			Vec2b posInGrid;
			GetPosInGrid(newRandNearPoint, posInGrid, gridScaleFactor);
			AddPoint2Grid(grid, newRandNearPoint, posInGrid, numSampledPoints);
			AddPointListNode(pointListNodePtr, newRandNearPoint, numSampledPoints);
			numSampledPoints++;
		}
		else
		{
			circle(img3C, randPointNode.point, 2, Scalar(255, 255, 255), -1);
			circle(emptyImg1C, randPointNode.point, 2, Scalar(255, 255, 255), -1);
			DelPointListNode(pointListNodePtr, randPointNode.sampleIndex);
		}
	}
	double t2 = ((double)getTickCount() - t1) / getTickFrequency();

	cout << "Number of seed points actually generated:" << numSampledPoints + 1 << endl;
	cout << "Elapsed time:" << t2 << endl;
	imshow("MISSION 1 RESULT", img3C);

	Mat watershedImg(imgMask.size(), CV_8UC3);

	//获得图像分界轮廓
	while (1)
	{
		vector<vector<Point>> contours;
		vector<Vec4i> Hier;
		int fillColorIndexMax = 0;
		findContours(emptyImg1C, contours, Hier, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
		if (contours.empty())
			continue;
		imgMask = Scalar::all(0);
		for (int contourIndex = 0; contourIndex >= 0; contourIndex = Hier[contourIndex][0], fillColorIndexMax++)
			drawContours(imgMask, contours, contourIndex, Scalar::all((double)fillColorIndexMax + 1), -1, 8, Hier, INT_MAX);
		if (fillColorIndexMax == 0)
			continue;
		vector<Vec3b> colorTable;
		for (int i = 0; i < fillColorIndexMax; i++)
		{
			uchar b = theRNG().uniform(0, 255);
			uchar g = theRNG().uniform(0, 255);
			uchar r = theRNG().uniform(0, 255);
			colorTable.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
		}
		watershed(img2watershed, imgMask);
		for (int i = 0; i < imgMask.rows; i++)
			for (int j = 0; j < imgMask.cols; j++)
			{
				int fillColorIndex = imgMask.at<int>(i, j);
				if (fillColorIndex == -1)
					watershedImg.at<Vec3b>(i, j) = Vec3b(255, 255, 255); //轮廓着白色
				else if (fillColorIndex <= 0 || fillColorIndex > fillColorIndexMax)
					watershedImg.at<Vec3b>(i, j) = Vec3b(0, 0, 0); //啥玩意
				else
					watershedImg.at<Vec3b>(i, j) = colorTable[(double)fillColorIndex - 1]; //区域着色
			}
		watershedImg = watershedImg * 0.5 + grayImg3C * 0.5;
		imshow("watershed RESULT", watershedImg);
#ifdef STEP3
		cout << "\nClick [MISSION 1 RESULT], press enter to get in MISSION 2." << endl;
#endif
		cout << "\nClick [MISSION 1 RESULT], press enter to get in MISSION 3." << endl;
		waitKey(0);
		break;
	}

	//MISSION 2：四原色填充
	AdjacentTable adjacentTable; //邻接表
#ifdef STEP3
	cout << "\n[MISSION 2]" << endl;
#endif
	//生成邻接表
	t1 = (double)getTickCount();
	GenerateAdjacentTable(adjacentTable, imgMask, numSampledPoints);
#ifdef STEP3
	//展示邻接表
	//ShowAdjacencyList(adjacentTable);

	Queue pathQueue; //生成路径队列
	pathQueue.front = NULL;
	pathQueue.rear = NULL;
	ChoosePath(adjacentTable, pathQueue); //生成生成路径队列
	t2 = ((double)getTickCount() - t1) / getTickFrequency();
	cout << "Elapsed time:" << t2 << endl;

	//生成四原色图
	GetFourColorsImg(adjacentTable, pathQueue, imgMask, highlightedImg);
	cout << "\nClick [MISSION 1 RESULT], press enter to get in MISSION 2." << endl;
	waitKey(0);
#endif

	//MISSION 3：折半查找
	cout << "\n[MISSION 3]" << endl;

	//按区域面积大小堆排序
	t1 = (double)getTickCount();
	HeapNode heap[MAX_RIGION_NUM + 1];
	HeapInit(heap);
	CaluHeapArea(imgMask, heap);

	HeapSortAsc(heap, numSampledPoints);

	t2 = ((double)getTickCount() - t1) / getTickFrequency();

	watershedImg = (watershedImg - grayImg3C * 0.5) * 2;
	while (1)
	{
		if (BinarySearch(heap, numSampledPoints, imgMask, watershedImg, adjacentTable, t2) == EXIT)
			break;
	}

	cout << "\nEnd" << endl;
	return 0;
}