#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

#define ERROR 0
#define OK 1
#define EXIT 2
#define NONE -1
#define TRUE 1
#define FALSE 0
#define INF 100000000
#define MAX_AREA_NUM MAX_VERTEX_NUM
#define SIBLING_SEPARATION 10
#define MAX_VERTEX_NUM 1200
#define RED 0
#define GREEN 1
#define BLUE 2
#define YELLOW 3
#define NOT_TINTED -1
#define M 980
#define N 635
#define PI 3.1415926
#define NODE_RADIUS 8
#define IMG_HEIGHT 600
#define LEFT 1
#define RIGHT 2

typedef struct actpoint
{
	Point actpointt;
	int acin;
	struct actpoint *next;
} actpoint;
typedef struct duijiedian
{
	unsigned int duimianji;
	int duishenfen;
} duijiedian;
typedef struct arcpoint
{
	int arcxuhao;
	int adjxuhao;
	int quanzhi1;
	unsigned int cdist;
	struct arcpoint *a_next;
} arcpoint;
typedef struct verpoint
{
	int vser_num;
	int adjxuhao;
	Point gross;
	arcpoint* archead;
} verpoint, liadj[MAX_VERTEX_NUM];
typedef struct ajaclist
{
	liadj versy;
	int texnums, arnums1;
} ajaclist;
typedef struct QNode
{
	int data;
	bool yansesele[4];
	struct QNode *next;
} QNode, *QueuePtr;
typedef struct LinkQueue
{
	QueuePtr front;
	QueuePtr rear;
} LinkQueue;
typedef struct colstas
{
	int quyuhao;
	int color;
	struct colstas *next;
} colstas;

int gchushi(Mat &Wgarray, double stepk, int &samnumsjud, actpoint *acts, int r);
int cposigain(Point pointer1, Vec2b &ceposi, double stepk);
int actchushi(actpoint *&acts);
int actlarge(actpoint *p, Point pointer1, int sind);
int actdel(actpoint *p, int sind);
int pgain(Mat Wgarray, actpoint ref_pt, Point &result_pt, int ilimtat, int r, double stepk);
int judgepv(Point pointer1, int r, double stepk, Mat Wgarray);
int ljugain(Mat Wgarray, Vec2b ceposi, Mat &lju);
int actcdu(actpoint *p);
int actromgain(actpoint *acts, actpoint &ref_pt);
int slarge(Mat &Wgarray, Point result_pt, Vec2b ceposi, int sind); 
int hlight1(duijiedian *heap, unsigned int inindex, unsigned int suind, Mat maskImage, Mat hlightd, ajaclist allim);
int zhebansearch(duijiedian *heap, int kcount, Mat maskImage, Mat hlightd, ajaclist allim);
int heapcs(duijiedian *heap);														 
int squadcs(LinkQueue &Q);															 
int entersquad(LinkQueue &Q, int e);												 
int judgeskong(LinkQueue Q);														 
int leavesquad(LinkQueue &Q, int &e);												 
int colorcssta(colstas *(&s));														 
int colorstalen(colstas *s);														 
int colorpush(colstas *(&s), int quyuhao, int color);								
int colorpop(colstas *(&s), int *quyuhao);											 
int listadj(ajaclist &allim, Mat maskImage, int kcount);							 
int showadj(ajaclist allim);														 
int dyecolor(ajaclist &allim, LinkQueue tinting_queue, Mat maskImage, Mat &hlightd); 
int selectdyecolor(ajaclist allim, const int *visited);								 
int detrow(ajaclist allim, int vser_num, const int *visited);						 
int judgevisit(ajaclist allim, const int *visited);									 
int optimize(ajaclist &allim);														 
int pathchoose(ajaclist allim, LinkQueue &tinting_queue);							 
int mianji(ajaclist &allim, Mat maskImage, Mat four_primary_colors_image);			 
int quanzhi(ajaclist &allim, Mat maskImage, int kcount, duijiedian *heap);			 
int sortlarge(duijiedian *heap, unsigned int mianjixu);								
int sortdes(duijiedian *heap, unsigned int mianjixu);								
int dagendui(duijiedian *heap, unsigned int mianjixu);
int xiaogendui(duijiedian *heap, unsigned int mianjixu);
int Maxscheng(duijiedian *heap, int specific, unsigned int mianjixu);
int Minscheng(duijiedian *heap, int specific, unsigned int mianjixu);
