#ifdef _CH_
#pragma package < opencv>
#endif

#include "stdafx.h"
#include "opencv.hpp"
#include "highgui/highgui.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#define PI 3.1415926

struct LinkedList
{
	float x;
	float y;
	int flag;
	double a0;
	LinkedList *next;
};
IplImage *marker_mask = 0;
IplImage *markers = 0;
IplImage *img0 = 0, *img = 0, *img_gray = 0, *wshed = 0;
CvPoint prev_pt = {-1, -1};
int M;
int N;
int poisson(int K);
float s(float x, float y, float x0, float y0);
void fsampleseed(struct LinkedList *ph, struct LinkedList *temp, int K);
void try2find(float *dx, float *dy, int K, double a);
void drawseed(struct LinkedList *ph);
int main(int argc, char **argv)
{
	char *filename = argc >= 2 ? argv[1] : (char *)"colors.jpg";
	CvRNG rng = cvRNG(-1);
	int K = 0;

	if ((img0 = cvLoadImage(filename, 1)) == 0)
		return 0;
	cvNamedWindow("image", 1);				 //������
	cvNamedWindow("watershed transform", 1); //������
	img = cvCloneImage(img0);
	img_gray = cvCloneImage(img0);
	wshed = cvCloneImage(img0);
	marker_mask = cvCreateImage(cvGetSize(img), 8, 1);
	markers = cvCreateImage(cvGetSize(img), IPL_DEPTH_32S, 1);
	cvCvtColor(img, marker_mask, CV_BGR2GRAY);
	cvCvtColor(marker_mask, img_gray, CV_GRAY2BGR);

	cvZero(marker_mask);
	cvZero(wshed);
	cvShowImage("image", img);
	cvShowImage("watershed transform", wshed);

	M = img->width;
	N = img->height;
	printf("tips: \n"
		   "\tENTER - run watershed algorithm\n");
	printf("Please input K:");
	scanf("%d", &K);
	poisson(K);
	CvMemStorage *storage = cvCreateMemStorage(0);
	CvSeq *contours = 0;
	CvMat *color_tab;
	int i, j, comp_count = 0;
	cvFindContours(marker_mask, storage, &contours, sizeof(CvContour),
				   CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	cvZero(markers);
	for (; contours != 0; contours = contours->h_next, comp_count++)
	{
		cvDrawContours(markers, contours, cvScalarAll(comp_count + 1),
					   cvScalarAll(comp_count + 1), -1, -1, 8, cvPoint(0, 0));
	}
	cvWatershed(img0, markers);
	color_tab = cvCreateMat(1, comp_count, CV_8UC3);
	for (i = 0; i < comp_count; i++)
	{
		uchar *ptr = color_tab->data.ptr + i * 3;
		ptr[0] = (uchar)(cvRandInt(&rng) % 180 + 50);
		ptr[1] = (uchar)(cvRandInt(&rng) % 180 + 50);
		ptr[2] = (uchar)(cvRandInt(&rng) % 180 + 50);
	}
	for (i = 0; i < markers->height; i++)
		for (j = 0; j < markers->width; j++)
		{
			int idx = CV_IMAGE_ELEM(markers, int, i, j);
			uchar *dst = &CV_IMAGE_ELEM(wshed, uchar, i, j * 3);
			if (idx == -1)
				dst[0] = dst[1] = dst[2] = (uchar)255;
			else if (idx <= 0 || idx > comp_count)
				dst[0] = dst[1] = dst[2] = (uchar)0; // should not get here
			else
			{
				uchar *ptr = color_tab->data.ptr + (idx - 1) * 3;
				dst[0] = ptr[0];
				dst[1] = ptr[1];
				dst[2] = ptr[2];
			}
		}

	cvAddWeighted(wshed, 0.5, img_gray, 0.5, 0, wshed);
	cvShowImage("watershed transform", wshed);
	cvShowImage("image", img);
	cvReleaseMemStorage(&storage);
	cvReleaseMat(&color_tab);
	cvWaitKey();
	return 0;
}
/*************************************************
�²���Բ��
���ܣ����ղ���Բ��ȡ���ӵ�
������
			int K���û�Ҫȡ�����ӵ����
*************************************************/
int poisson(int K)
{
	int i;						  //�����������ӵ�ĸ���
	struct LinkedList *ph, *temp; //���ӵ�����ͷ��� ��ʱָ��
	CvRNG rng;					  //�����

	rng = cvRNG(cvGetTickCount());								 //��ʼ��
	ph = (struct LinkedList *)malloc(sizeof(struct LinkedList)); //��������
	temp = ph;													 //��ʱָ����ָ��ͷ���
	ph->x = (float)M / 2;										 //��һ�����ӵ��λ��
	ph->y = (float)cvSqrt((float)(M * N / K));
	ph->flag = 0; //��ʼ����һ�����ӵ��־λ
	ph->next = NULL;
	ph->a0 = 2 * PI * cvRandReal(&rng);		//���ȡ���ʼ�Ƕ�
	for (i = 1; i < K && temp != NULL; i++) //������ĵ�
	{
		fsampleseed(ph, temp, K); //�ҵ����ӵ�
		if (temp->flag == 1)	  //���ȷ�ϸ����ӵ���Χ�Ѿ�û�п�ȡ�ĵ�
		{
			i--;			   //�Ƚ�����λ�Լ�ʹ����λ�������
			temp = temp->next; //��ʼ����һ�����ӵ����
		}
	}
	drawseed(ph); //�������ӵ�
	return 0;
}
/*************************************************
Ѱ�����ӵ�
���ܣ������ҵ���ǰ���ӵ���Χ��ȡ�����ӵ㣬����������
������
			struct LinkedList*ph������ͷ���
			struct LinkedList*tem����ǰ���������ӵ�
			int K�����ӵ����
*************************************************/
void fsampleseed(struct LinkedList *ph, struct LinkedList *temp, int K)
{
	float *dx, *dy;					  //�����ӵ㵽��ǰ���ӵ�ľ���
	int i = 0;						  //������Ҵ���
	struct LinkedList *p, *add, *end; //��ʱָ�� ���ӵĽ�� ��β���
	double a;						  //�Ƕ�
	CvRNG rng;						  //�����

	rng = cvRNG(cvGetTickCount());								  //��ʼ�������
	add = (struct LinkedList *)malloc(sizeof(struct LinkedList)); //��ʼ��
	dx = (float *)malloc(sizeof(float));
	dy = (float *)malloc(sizeof(float));
	for (end = ph; end->next != NULL; end = end->next)
		;						  //ָ��ĩλ���
	a = temp->a0;				  //��ʼ�Ƕ�
	while (a < temp->a0 + 2 * PI) //��ǰ���ӵ���ΧһȦû�б�����
	{
		do
		{
			if (i < 3) //�������
			{
				i++;																//����
				try2find(dx, dy, K, temp->a0 + (cvRandInt(&rng) % 360) * PI / 180); //�������һ����
			}
			else //������ҽ�����ʼ����
			{
				a += PI / 180;			//ÿ������һ���Ƕ�
				try2find(dx, dy, K, a); //�Ե�ǰ�Ƕȳ���
			}
		} while (((temp->x + *dx) <= 0) || ((temp->x + *dx) >= (float)M) || ((temp->y + *dy) <= 0) || ((temp->y + *dy) >= (float)N)); //�õ㻹��ͼ��Χ��
		for (p = ph; p->next != NULL; p = p->next)																					  //��������
			if (s(temp->x + *dx, temp->y + *dy, p->x, p->y) < (float)(M * N / K))													  //�ж��Ƿ��ÿ���㶼��������
				break;
		if (p->next == NULL && (s(temp->x + *dx, temp->y + *dy, p->x, p->y) >= (float)(M * N / K))) //������������Ҷ���������
		{
			add->x = temp->x + *dx; //���ýڵ������������ʼ��
			add->y = temp->y + *dy;
			add->flag = 0;
			add->a0 = PI * cvFastArctan(-*dx, -*dy) / 180; //��ʼ�Ƕ�λ��Ϊ����ǰ���ӵ�
			p->next = add;
			add->next = NULL;
			break;
		}
	}
	if (a >= temp->a0 + 2 * PI) //���ȷ�����Ҳ����µ����ӵ�
	{
		temp->flag = 1; //����ǰ���ӵ���λ
		free(add);
	}
	free(dx);
	free(dy);
}
/*************************************************
���뺯��
���ܣ��������������ƽ��
*************************************************/
float s(float x, float y, float x0, float y0)
{
	return (x - x0) * (x - x0) + (y - y0) * (y - y0);
}
/*************************************************
���ܣ����㳢�ԽǶȵĵ������
*************************************************/
void try2find(float *dx, float *dy, int K, double a)
{
	double r;
	//CvRNG rng;

	//rng= cvRNG(cvGetTickCount());
	r = cvSqrt((float)(M * N / K)); //�̶��뾶ΪҪ�����С����
	*dx = (float)((r)*cos(a));		//
	*dy = (float)((r)*sin(a));
	//*a=(cvRandInt(&rng)%360)*PI/180;
	//*a+=PI/180;
}
/*************************************************
���ӵ���ƺ���
���ܣ���ͼ�Ͻ����ӵ㻭��
*************************************************/
void drawseed(struct LinkedList *ph)
{
	struct LinkedList *temp;
	int t;
	char s[5];
	CvFont font;

	cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0, 2, 8);
	for (temp = ph, t = 1; temp != NULL; temp = temp->next, t++)
	{
		itoa(t, s, 10);
		cvRectangle(marker_mask, cvPoint((int)temp->x - 2, (int)temp->y - 2), cvPoint((int)temp->x + 1, (int)temp->y + 1), cvScalar(255, 255, 255), 3, 4, 0);
		cvPutText(img, s, cvPoint((int)temp->x + 1, (int)temp->y - 1), &font, cvScalar(0, 0, 0));
		cvRectangle(img, cvPoint((int)temp->x - 2, (int)temp->y - 2), cvPoint((int)temp->x + 1, (int)temp->y + 1), cvScalar(0, 0, 0), 3, 4, 0);
	}
	printf("total:%d\n", t - 1);
}
