#pragma once

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#define ERROR 0
#define OK 1
#define EXIT 2
#define NONE -1
#define FALSE 0
#define TRUE 1

#define RED 0
#define GREEN 1
#define BLUE 2
#define YELLOW 3

#define PI 3.1415926

#define GetNewNearPointListNodeIterateLimit 60
#define INF 100000000
#define MAX_RIGION_NUM 1500
#define POP_RECORDS_LIMIT 250

#define M 1024
#define N 664

#define IMGPATH ("D:\\default.jpeg")
#define ORIGINPOINT (M / 2, N / 2)

//#define STEP3 //用于控制是否进行四原色填充并输出可视化结果（由于四原色填充过程耗时较长，跳过该步骤可节省延时时间）