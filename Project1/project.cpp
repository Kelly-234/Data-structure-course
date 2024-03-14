#include "project.h"
int main()
{
	Mat image1, gray1, src2, src1 = imread("D:\\default.jpeg"), img = imread("D:\\default.jpeg");
	Mat mask1(src1.size(), CV_32S), gaoguang1(mask1.size(), CV_8UC3);
	int count = 0;
	duijiedian heap[MAX_AREA_NUM + 1];
	ajaclist AL;
	heapcs(heap);

	//MISSION 1：随机打点+分水岭
	int itlim = 100, samnums = 0, i, j, cop = 0, index, b, g, radius, glie, ghang;
	double K, wnum;
	actpoint *act = NULL;
	RNG rng((unsigned)time(NULL));
	src1.copyTo(src2);
	cvtColor(src1, image1, COLOR_BGR2GRAY);
	cvtColor(image1, gray1, COLOR_GRAY2BGR);
	image1 = Scalar::all(0);
	cout << "\n[MISSION 1]" << endl;
	while (1)
	{
		cout << "Number of seed points:";
		cin >> K;
		if (K != floor(K))
			cout << "Input error.(please input an integer)" << endl;
		else if (K <= 1 || K > 1000)
			cout << "Input error.(between 1~1001)" << endl;
		else
			break;
	}
	radius = (int)sqrt((double)M * N / K);
	wnum = radius / sqrt(2);
	glie = (int)(M / wnum) + 1;
	ghang = (int)(N / wnum) + 1;
	Mat grid(glie, ghang, CV_16SC3, Scalar(NONE, NONE, NONE));
	if (!actchushi(act) || !gchushi(grid, wnum, samnums, act, radius))
	{
		cout << "Error: run out of memory." << endl;
		exit(1);
	}
	while (actcdu(act))
	{
		actpoint rpt;
		Point rupt;
		actromgain(act, rpt);
		if (pgain(grid, rpt, rupt, itlim, radius, wnum))
		{
			Vec2b xiposi;
			cposigain(rupt, xiposi, wnum);
			slarge(grid, rupt, xiposi, samnums);
			actlarge(act, rupt, samnums);
			samnums++;
		}
		else
		{
			circle(img, rpt.actpointt, 2, Scalar(255, 255, 255), -1);	 //画点
			circle(image1, rpt.actpointt, 2, Scalar(255, 255, 255), -1); //不晓得
			actdel(act, rpt.acin);
			count++;
		}
	}
	cout << "Number of seed points actually generated:" << count << endl;
	imshow("MISSION 1 RESULT", img);
	while (1)
	{
		vector<vector<Point>> lunkuo;
		vector<Vec4i> Hier;
		findContours(image1, lunkuo, Hier, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
		if (lunkuo.empty())
			continue;
		mask1 = Scalar::all(0);
		for (index = 0; index >= 0; index = Hier[index][0], cop++)
			drawContours(mask1, lunkuo, index, Scalar::all((double)cop + 1), -1, 8, Hier, INT_MAX);
		if (cop == 0)
			continue;
		vector<Vec3b> colorTab;
		for (i = 0; i < cop; i++)
		{
			b = theRNG().uniform(0, 255);
			g = theRNG().uniform(0, 255);
			radius = theRNG().uniform(0, 255);
			colorTab.push_back(Vec3b((uchar)b, (uchar)g, (uchar)radius)); //获取随机着色的颜色值
		}
		watershed(src2, mask1);
		Mat wshedima(mask1.size(), CV_8UC3);
		for (i = 0; i < mask1.rows; i++)
			for (j = 0; j < mask1.cols; j++)
			{
				index = mask1.at<int>(i, j);
				if (index == -1)
					wshedima.at<Vec3b>(i, j) = Vec3b(255, 255, 255); //轮廓着白色
				else if (index <= 0 || index > cop)
					wshedima.at<Vec3b>(i, j) = Vec3b(0, 0, 0); //啥玩意
				else
					wshedima.at<Vec3b>(i, j) = colorTab[(double)index - 1]; //区域着色
			}
		wshedima = wshedima * 0.5 + gray1 * 0.5;
		imshow("watershed RESULT", wshedima);
		cout << "\nClick [MISSION 1 RESULT], press enter to get in MISSION 2." << endl;
		waitKey(0);
		break;
	}

	//MISSION 2：四原色填充
	Mat lunkuoima;
	LinkQueue linksquad;
	cout << "\n[MISSION 2]" << endl;
	listadj(AL, mask1, count);
	quanzhi(AL, mask1, count, heap);
	showadj(AL);
	linksquad.front = NULL;
	linksquad.rear = NULL;
	pathchoose(AL, linksquad);
	dyecolor(AL, linksquad, mask1, gaoguang1);
	cout << "\nClick [MISSION 1 RESULT], press enter to get in MISSION 3." << endl;
	waitKey(0);

	//堆排序
	Mat gaoguang1_copy1, gaoguang1_copy2;
	src2 = imread("D:\\default.jpeg");
	gaoguang1.copyTo(gaoguang1_copy1);
	gaoguang1.copyTo(gaoguang1_copy2);
	sortdes(heap, count);
	sortlarge(heap, count);

	//MISSION 3：折半查找
	cout << "\n[MISSION 3]" << endl;
	while (1)
	{
		if (zhebansearch(heap, count, mask1, gaoguang1, AL) == EXIT)
			break;
	}

	cout << "\nEnd" << endl;
	return 0;
}

int gchushi(Mat &grid, double wnum, int &samnums, actpoint *act, int radius)
{
	Point origWang(250, 250);
	Vec2b xiposi;
	short *data = grid.ptr<short>(xiposi[1]);
	cposigain(origWang, xiposi, wnum);
	data[xiposi[0] * grid.channels()] = origWang.x;
	data[xiposi[0] * grid.channels() + 1] = origWang.y;
	data[xiposi[0] * grid.channels() + 2] = 0;
	samnums += 1;
	actlarge(act, origWang, 0);
	return OK;
}
int cposigain(Point pointer1, Vec2b &xiposi, double wnum)
{
	xiposi[0] = (int)(pointer1.x / wnum);
	xiposi[1] = (int)(pointer1.y / wnum);
	return OK;
}
//初始化act指针
int actchushi(actpoint *&act)
{
	if ((act = (actpoint *)malloc(sizeof(actpoint))) == NULL)
		return ERROR;
	act->next = NULL;
	return OK;
}
int actlarge(actpoint *p, Point pointer1, int sind)
{
	actpoint *temp = NULL;
	while (p->next != NULL)
		p = p->next;
	if ((temp = (actpoint *)malloc(sizeof(actpoint))) == NULL)
		return ERROR;
	p->next = temp;
	temp->next = NULL;
	temp->actpointt = pointer1;
	temp->acin = sind;
	return OK;
}
int actdel(actpoint *p, int sind)
{
	actpoint *temp = p->next;
	if (p->next == NULL)
		return ERROR;
	while (temp->next != NULL && temp->acin != sind)
	{
		p = temp;
		temp = temp->next;
	}
	if (temp->acin == sind)
	{
		p->next = temp->next;
		free(temp);
		temp = NULL;
	}
	return OK;
}
//计算各点坐标
int pgain(Mat grid, actpoint rpt, Point &rupt, int itlim, int radius, double wnum)
{
	int i = 0;
	double angle = 0, rand_radius;
	RNG rng((unsigned)time(NULL));
	Point pointer1;
	while (i < itlim)
	{
		angle = rng.uniform((double)0, (double)(2 * PI));
		rand_radius = (double)radius + 1;
		pointer1.x = rpt.actpointt.x + (int)(rand_radius * cos(angle));
		pointer1.y = rpt.actpointt.y + (int)(rand_radius * sin(angle));
		if (pointer1.x < 0 || pointer1.x > M || pointer1.y < 0 || pointer1.y > N)
			continue;
		if (judgepv(pointer1, radius, wnum, grid))
		{
			rupt.x = pointer1.x;
			rupt.y = pointer1.y;
			return OK;
		}
		i++;
	}
	return ERROR; //已有点坐标？
}
//判断点坐标合法性
int judgepv(Point pointer1, int radius, double wnum, Mat grid)
{
	int i, j;
	double dist2;
	Mat lju(5, 5, CV_16SC3, Scalar(NONE, NONE, NONE));
	Point near;
	Vec2b xiposi;
	cposigain(pointer1, xiposi, wnum);
	ljugain(grid, xiposi, lju);
	for (i = 0; i < lju.rows; i++)
	{
		for (j = 0; j < lju.cols; j++)
		{
			short *data = lju.ptr<short>(i);
			if (data[j * lju.channels()] == NONE && data[j * lju.channels() + 1] == NONE && data[j * lju.channels() + 2] == NONE)
				continue;
			dist2 = pow((data[j * lju.channels()] - pointer1.x), 2) + pow((data[j * lju.channels() + 1] - pointer1.y), 2);
			if (dist2 < pow(radius, 2))
				return FALSE;
		}
	}
	return TRUE;
}
int ljugain(Mat grid, Vec2b xiposi, Mat &lju)
{
	int j = 0, i;
	int setff[25][2]{{-2, -2},{-1, -2},{0, -2},{1, -2},{2, -2},{-2, -1},{-1, -1},{0, -1},{1, -1},
		             {2, -1},{-2, 0},{-1, 0},{0, 0},{1, 0},{2, 0},{-2, 1},{-1, 1},{0, 1},
					 {1, 1},{2, 1},{-2, 2},{-1, 2},{0, 2},{1, 2},{2, 2},};
	for (i = 0; i < 25; i++)
	{
		if ((xiposi[0] + setff[i][0]) > -3 && (xiposi[0] + setff[i][0]) < grid.cols * grid.channels() && (xiposi[1] + setff[i][1] >= 0 && (xiposi[1] + setff[i][1]) < grid.rows))
		{
			short *gnumff = grid.ptr<short>(xiposi[1] + setff[i][1]);
			if (i % 5 == 0 && i != 0)
				j++;
			short *nnumff = lju.ptr<short>(j);
			nnumff[(i % 5) * lju.channels()] = gnumff[(xiposi[0] + setff[i][0]) * grid.channels()];
			nnumff[(i % 5) * lju.channels() + 1] = gnumff[(xiposi[0] + setff[i][0]) * grid.channels() + 1];
			nnumff[(i % 5) * lju.channels() + 2] = gnumff[(xiposi[0] + setff[i][0]) * grid.channels() + 2];
		}
		else
			continue;
	}
	return OK;
}
int actcdu(actpoint *p)
{
	int length = 0;
	while (p->next != NULL)
	{
		length++;
		p = p->next;
	}
	return length;
}
int actromgain(actpoint *act, actpoint &rpt)
{
	RNG rng((unsigned)time(NULL));
	int i, Rinds = rng.uniform(0, actcdu(act));
	for (i = 0; i < Rinds; i++)
		act = act->next;
	rpt.actpointt = act->next->actpointt;
	rpt.acin = act->next->acin;
	return OK;
}
int slarge(Mat &grid, Point rupt, Vec2b xiposi, int sind)
{
	short *data = grid.ptr<short>(xiposi[1]);
	data[xiposi[0] * grid.channels()] = rupt.x;
	data[xiposi[0] * grid.channels() + 1] = rupt.y;
	data[xiposi[0] * grid.channels() + 2] = sind;
	return OK;
}
int mianji(ajaclist &AL, Mat mask1, Mat four_primary_colors_image)
{
	for (int i = 0; i < AL.texnums; i++)
	{
		int centroid_x = 0, centroid_y = 0, number = 0;
		for (int j = 1; j < mask1.rows - 1; j++)
		{
			int *data = mask1.ptr<int>(j);
			for (int k = 1; k < mask1.cols - 1; k++)
			{
				if (data[k] == i + 1)
				{
					centroid_x += k;
					centroid_y += j;
					number++;
				}
			}
		}
		if (!number)
			number = 1;
		AL.versy[i].gross.x = centroid_x / number;
		AL.versy[i].gross.y = centroid_y / number;
	}
	for (int i = 0; i < AL.texnums; i++)
	{
		arcpoint *temp = AL.versy[i].archead;
		while (temp != NULL)
		{
			temp->cdist = (unsigned int)sqrt(pow((AL.versy[i].gross.x - AL.versy[temp->arcxuhao - 1].gross.x), 2) + pow((AL.versy[i].gross.y - AL.versy[temp->arcxuhao - 1].gross.y), 2));
			temp = temp->a_next;
		}
	}
	for (int i = 0; i < AL.texnums; i++)
	{
		char str[10];
		sprintf_s(str, "%d", i + 1);
		putText(four_primary_colors_image, str, AL.versy[i].gross, FONT_HERSHEY_PLAIN, 0.8, Scalar(255, 255, 255));
	}
	return OK;
}
int quanzhi(ajaclist &AL, Mat mask1, int count, duijiedian *heap)
{
	int i, j;
	for (i = 1; i < mask1.rows - 1; i++)
	{
		int *data = mask1.ptr<int>(i);
		for (j = 1; j < mask1.cols - 1; j++)
		{
			if (data[j] != -1 && data[j] > 0)
				heap[data[j]].duimianji++;
		}
	}
	for (i = 0; i < AL.texnums; i++)
	{
		arcpoint *temp = AL.versy[i].archead;
		while (temp != NULL)
		{
			temp->quanzhi1 = heap[temp->arcxuhao].duimianji;
			temp = temp->a_next;
		}
	}
	return OK;
}
int squadcs(LinkQueue &Q)
{
	Q.front = Q.rear = (QueuePtr)malloc(sizeof(QNode));
	if (!Q.front)
		return ERROR;
	Q.front->next = NULL;
	return OK;
}
int entersquad(LinkQueue &Q, int e)
{
	QueuePtr s;
	if ((s = (QueuePtr)malloc(sizeof(QNode))) == NULL)
		return ERROR;
	s->data = e;
	s->next = NULL;
	Q.rear->next = s;
	Q.rear = s;
	return OK;
}
int judgeskong(LinkQueue Q)
{
	if (Q.front->next == NULL)
		return TRUE;
	else
		return FALSE;
}
int leavesquad(LinkQueue &Q, int &e)
{
	QueuePtr p;
	if (Q.front == Q.rear)
		return ERROR;

	p = Q.front->next;
	e = p->data;
	Q.front->next = p->next;
	if (p == Q.rear)
		Q.rear = Q.front;
	free(p);
	p = NULL;
	return OK;
}
int colorcssta(colstas *(&s))
{
	if ((s = (colstas *)malloc(sizeof(colstas))) == NULL)
		return ERROR;
	s->next = NULL;
	return OK;
}
int colorstalen(colstas *s)
{
	int length = 0;
	colstas *temp = s;
	while (temp != NULL)
	{
		length++;
		temp = temp->next;
	}
	return length - 1;
}
int colorpush(colstas *(&s), int quyuhao, int color)
{
	colstas *p;
	if ((p = (colstas *)malloc(sizeof(colstas))) == NULL)
		return ERROR;
	p->quyuhao = quyuhao;
	p->color = color;
	p->next = s;
	s = p;
	return OK;
}
int colorpop(colstas *(&s), int *quyuhao)
{
	colstas *p;
	if (s->next != NULL)
	{
		p = s;
		*quyuhao = s->quyuhao;
		s = s->next;
		free(p);
		p = NULL;
		return OK;
	}
	else
		return ERROR;
}
int listadj(ajaclist &AL, Mat mask1, int count)
{
	int arnums1 = 0, i, j, k, area1_ID, area2_ID, edge[8], ref;
	AL.texnums = count;
	for (i = 0; i < count; i++) //分配区域编号
	{
		AL.versy[i].vser_num = i + 1;
		AL.versy[i].archead = NULL;
	}
	for (i = 1; i < mask1.rows - 1; i++)
	{
		int *data1 = mask1.ptr<int>(i - 1);
		int *data2 = mask1.ptr<int>(i);
		int *data3 = mask1.ptr<int>(i + 1);
		for (j = 1; j < mask1.cols - 1; j++)
		{
			ref = data2[j];
			edge[0] = data1[j - 1];
			edge[1] = data1[j + 1];
			edge[2] = data1[j + 1];
			edge[3] = data2[j - 1];
			edge[4] = data2[j + 1];
			edge[5] = data3[j - 1];
			edge[6] = data3[j];
			edge[7] = data3[j + 1];
			if (ref == -1) //在干嘛
			{
				for (k = 0; k < 8; k++)
				{
					if (edge[k] > 0)
					{
						area1_ID = edge[k];
						break;
					}
				}
				for (k = 0; k < 8; k++)
				{
					if (edge[k] > 0 && edge[k] != area1_ID)
					{
						area2_ID = edge[k];
						break;
					}
				}
				if (AL.versy[area1_ID - 1].archead == NULL)
				{
					arcpoint *p = NULL;
					if ((p = (arcpoint *)malloc(sizeof(arcpoint))) == NULL)
					{
						cout << "Error: insufficient memory..." << endl;
						exit(1);
					}
					p->a_next = NULL;
					p->arcxuhao = area2_ID;
					AL.versy[area1_ID - 1].archead = p;
					arnums1++;
				}
				else if (AL.versy[area1_ID - 1].archead != NULL)
				{
					int IsDuplicate = FALSE;
					arcpoint *p = NULL;
					arcpoint *temp = AL.versy[area1_ID - 1].archead;
					while (temp != NULL)
					{
						if (temp->arcxuhao != area2_ID && temp->a_next != NULL)
							temp = temp->a_next;
						else if (temp->arcxuhao == area2_ID || area2_ID == AL.versy[area1_ID - 1].vser_num)
						{
							IsDuplicate = TRUE;
							break;
						}
						else
							break;
					}
					if (!IsDuplicate)
					{
						if ((p = (arcpoint *)malloc(sizeof(arcpoint))) == NULL)
						{
							cout << "Error: insufficient memory..." << endl;
							exit(1);
						}
						p->arcxuhao = area2_ID;
						p->a_next = NULL;
						temp->a_next = p;
						arnums1++;
					}
				}
				if (AL.versy[area2_ID - 1].archead == NULL)
				{
					arcpoint *p = NULL;
					if ((p = (arcpoint *)malloc(sizeof(arcpoint))) == NULL)
					{
						cout << "Error: insufficient memory..." << endl;
						exit(1);
					}
					p->a_next = NULL;
					p->arcxuhao = area1_ID;
					AL.versy[area2_ID - 1].archead = p;
					arnums1++;
				}
				else if (AL.versy[area2_ID - 1].archead != NULL)
				{
					int IsDuplicate = FALSE;
					arcpoint *p = NULL;
					arcpoint *temp = AL.versy[area2_ID - 1].archead;
					while (temp != NULL)
					{
						if (temp->arcxuhao != area1_ID && temp->a_next != NULL)
							temp = temp->a_next;
						else if (temp->arcxuhao == area1_ID || area1_ID == AL.versy[area2_ID - 1].vser_num)
						{
							IsDuplicate = TRUE;
							break;
						}
						else
							break;
					}
					if (!IsDuplicate)
					{
						if ((p = (arcpoint *)malloc(sizeof(arcpoint))) == NULL)
						{
							cout << "Error: insufficient memory..." << endl;
							exit(1);
						}
						p->arcxuhao = area1_ID;
						p->a_next = NULL;
						temp->a_next = p;
						arnums1++;
					}
				}
			}
		}
	}
	optimize(AL);
	AL.arnums1 = arnums1 / 2;
	return OK;
}
//生成邻接表？
int optimize(ajaclist &AL)
{
	int length = 1, i, j, k;
	int adj_area_number_list[MAX_VERTEX_NUM] = {0};
	for (i = 0; i < AL.texnums; i++)
	{
		arcpoint *temp = AL.versy[i].archead;
		while (temp != NULL)
		{
			length++;
			temp = temp->a_next;
		}
		adj_area_number_list[i] = length;
		length = 1;
	}
	for (i = 0; i < AL.texnums; i++)
	{
		AL.versy[i].adjxuhao = adj_area_number_list[i];
		arcpoint *temp = AL.versy[i].archead;
		while (temp != NULL)
		{
			temp->adjxuhao = adj_area_number_list[temp->arcxuhao - 1];
			temp = temp->a_next;
		}
	}
	for (i = 0; i < AL.texnums; i++)
	{
		for (j = 0; j < adj_area_number_list[i] - 2; j++)
		{
			arcpoint *temp = AL.versy[i].archead;
			arcpoint *shadow = NULL;
			for (k = 0; k < adj_area_number_list[i] - j - 2; k++)
			{
				if (temp->adjxuhao < temp->a_next->adjxuhao)
				{
					if (AL.versy[i].archead == temp)
					{
						arcpoint *current = temp->a_next;
						temp->a_next = temp->a_next->a_next;
						current->a_next = temp;
						AL.versy[i].archead = current;
						shadow = current;
					}
					else
					{
						arcpoint *current = temp->a_next;
						temp->a_next = temp->a_next->a_next;
						current->a_next = temp;
						shadow->a_next = current;
						shadow = current;
					}
				}
				else
				{
					shadow = temp;
					temp = temp->a_next;
				}
			}
		}
	}
	return OK;
}
//输出邻接表
int showadj(ajaclist AL)
{
	cout << "[Adjacency List]" << endl;
	for (int i = 0; i < AL.texnums; i++)
	{
		arcpoint *temp = AL.versy[i].archead;
		if (temp == NULL)
			continue;
		cout << AL.versy[i].vser_num << "-";
		while (temp != NULL && temp->a_next != NULL)
		{
			cout << temp->arcxuhao << "-";
			temp = temp->a_next;
		}
		if (temp != NULL && temp->a_next == NULL)
			cout << temp->arcxuhao << endl;
	}
	return OK;
}
//涂色并输出图片
int dyecolor(ajaclist &AL, LinkQueue linksquad, Mat mask1, Mat &gaoguang1)
{
	int pop_record[MAX_VERTEX_NUM] = {0};
	int colors_used_matrix[MAX_VERTEX_NUM][4] = {0};
	LinkQueue buffer;
	LinkQueue transfer_station;
	squadcs(buffer);
	squadcs(transfer_station);
	colstas *stack = NULL;
	colorcssta(stack);
	int e;
	int count = 0;
	while (!judgeskong(linksquad) || !judgeskong(buffer))
	{
		for (int i = 0; i < AL.texnums; i++)
		{
			if (pop_record[i] >= 250)
			{
				pop_record[i] = 0;
				for (int j = 0; j < 20; j++)
				{
					int temp_area_number;
					colorpop(stack, &temp_area_number);
					entersquad(transfer_station, temp_area_number);
					while (!judgeskong(buffer))
					{
						int transfer;
						leavesquad(buffer, transfer);
						entersquad(transfer_station, transfer);
					}
					while (!judgeskong(transfer_station))
					{
						int transfer;
						leavesquad(transfer_station, transfer);
						entersquad(buffer, transfer);
					}
				}
			}
		}
		if (!judgeskong(buffer))
		{
			bool left_colors_table[4] = {0};
			leavesquad(buffer, e);
			if (colorstalen(stack))
			{
				colstas *search_stack = stack;
				while (search_stack != NULL)
				{
					arcpoint *temp = AL.versy[e - 1].archead;
					while (temp != NULL)
					{
						if (temp->arcxuhao == search_stack->quyuhao)
							left_colors_table[search_stack->color] = 1;
						temp = temp->a_next;
					}
					search_stack = search_stack->next;
				}
				int tinted = FALSE;
				for (int i = 0; i < 4; i++)
				{
					if (left_colors_table[i] != 1 && colors_used_matrix[e - 1][i] == 0)
					{
						colorpush(stack, e, i);
						colors_used_matrix[e - 1][i] = 1;
						tinted = TRUE;
						break;
					}
				}
				if (tinted == FALSE)
				{
					for (int i = 0; i < 4; i++)
						colors_used_matrix[e - 1][i] = 0;

					int temp_area_number;
					colorpop(stack, &temp_area_number);
					pop_record[temp_area_number - 1]++;
					entersquad(transfer_station, temp_area_number);
					entersquad(transfer_station, e);
					while (!judgeskong(buffer))
					{
						int transfer;
						leavesquad(buffer, transfer);
						entersquad(transfer_station, transfer);
					}
					while (!judgeskong(transfer_station))
					{
						int transfer;
						leavesquad(transfer_station, transfer);
						entersquad(buffer, transfer);
					}
				}
			}
			else
			{
				int flag = FALSE;
				for (int i = 0; i < 4; i++)
				{
					if (colors_used_matrix[e - 1][i] != 1)
					{
						colorpush(stack, e, i);
						colors_used_matrix[e - 1][i] = 1;
						flag = TRUE;
						break;
					}
				}
				if (flag == FALSE)
					cout << "Coloring failure!" << endl;
			}
		}
		else if (judgeskong(buffer))
		{
			leavesquad(linksquad, e);
			bool left_colors_table[4] = {0};
			if (colorstalen(stack))
			{
				colstas *search_stack = stack;
				while (search_stack != NULL)
				{
					arcpoint *temp = AL.versy[e - 1].archead;
					while (temp != NULL)
					{
						if (temp->arcxuhao == search_stack->quyuhao)
							left_colors_table[search_stack->color] = 1;
						temp = temp->a_next;
					}
					search_stack = search_stack->next;
				}
				int tinted = FALSE;
				for (int i = 0; i < 4; i++)
				{
					if (left_colors_table[i] != 1)
					{
						colorpush(stack, e, i);
						colors_used_matrix[e - 1][i] = 1;
						tinted = TRUE;
						break;
					}
				}
				if (tinted == FALSE)
				{
					int temp_area_number;
					colorpop(stack, &temp_area_number);
					pop_record[temp_area_number]++;
					entersquad(buffer, temp_area_number);
					entersquad(buffer, e);
				}
			}
			else
			{
				colorpush(stack, e, RED);
				colors_used_matrix[e - 1][RED] = 1;
			}
		}
	}
	Mat src2 = imread("D:\\default.jpeg");
	Mat gray1;
	cvtColor(src2, gray1, COLOR_BGR2GRAY);
	cvtColor(gray1, gray1, COLOR_GRAY2BGR);
	Mat four_primary_colors_image(mask1.size(), CV_8UC3);
	for (int i = 0; i < mask1.rows; i++)
		for (int j = 0; j < mask1.cols; j++)
		{
			int index = mask1.at<int>(i, j);
			colstas *temp = stack;
			if (index > 0)
			{
				while (temp->next != NULL)
				{
					if (temp->quyuhao == index)
					{
						switch (temp->color)
						{
						case RED:
							four_primary_colors_image.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
							gaoguang1.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
							break;
						case GREEN:
							four_primary_colors_image.at<Vec3b>(i, j) = Vec3b(0, 255, 0);
							gaoguang1.at<Vec3b>(i, j) = Vec3b(0, 255, 0);
							break;
						case BLUE:
							four_primary_colors_image.at<Vec3b>(i, j) = Vec3b(255, 0, 0);
							gaoguang1.at<Vec3b>(i, j) = Vec3b(255, 0, 0);
							break;
						case YELLOW:
							four_primary_colors_image.at<Vec3b>(i, j) = Vec3b(0, 255, 255);
							gaoguang1.at<Vec3b>(i, j) = Vec3b(0, 255, 255);
							break;
						}
					}
					temp = temp->next;
				}
			}
			else if (index == -1)
			{
				four_primary_colors_image.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
				gaoguang1.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
			}
		}
	four_primary_colors_image = four_primary_colors_image * 0.5 + gray1 * 0.5;
	mianji(AL, mask1, four_primary_colors_image);
	imshow("[four primary colour]", four_primary_colors_image);
	return OK;
}
int detrow(ajaclist AL, int vser_num, const int *visited)
{
	int length = 0;
	if (visited[AL.versy[vser_num - 1].vser_num] == FALSE)
		length++;
	arcpoint *temp = AL.versy[vser_num - 1].archead;
	if (temp == NULL)
		return length;
	while (temp != NULL)
	{
		if (visited[temp->arcxuhao - 1] == FALSE)
			length++;
		temp = temp->a_next;
	}
	return length;
}
//分配各区域颜色？
int selectdyecolor(ajaclist AL, const int *visited)
{
	int index = NONE, length = 0, i, temp;
	for (i = 0; i < AL.texnums; i++)
	{
		if (visited[i] == FALSE)
		{
			temp = detrow(AL, i + 1, visited);
			if (temp > length)
			{
				index = i;
				length = temp;
			}
		}
	}
	if (index == NONE)
		return ERROR;
	else
		return index;
}
int judgevisit(ajaclist AL, const int *visited)
{
	for (int i = 0; i < AL.texnums; i++)
	{
		if (visited[i] == FALSE)
			return FALSE;
	}
	return TRUE;
}
//寻找并输出最佳涂色路线
int pathchoose(ajaclist AL, LinkQueue &linksquad)
{
	LinkQueue Q;
	int visited[MAX_VERTEX_NUM] = {FALSE}, i, u, v;
	//寻找最短路线
	for (i = 0; i < AL.texnums; i++) //获取邻接表长度？
	{
		arcpoint *temp = AL.versy[i].archead;
		if (temp == NULL)
			visited[i] = TRUE;
	}
	squadcs(linksquad);
	squadcs(Q);
	while (!judgevisit(AL, visited))
	{
		v = selectdyecolor(AL, visited);
		if (!visited[v])
		{
			visited[v] = TRUE;

			entersquad(linksquad, v + 1);
			entersquad(Q, v);
			while (!judgeskong(Q))
			{
				leavesquad(Q, u);
				arcpoint *temp = AL.versy[u].archead;
				if (temp == NULL)
					break;
				while (temp != NULL)
				{
					if (!visited[(temp->arcxuhao) - 1])
					{
						visited[(temp->arcxuhao) - 1] = TRUE;
						entersquad(linksquad, temp->arcxuhao);
						entersquad(Q, (temp->arcxuhao) - 1);
					}
					temp = temp->a_next;
				}
			}
		}
	}
	//输出路线
	cout << "\nThe path is:" << endl;
	LinkQueue temp = linksquad;
	temp.front = temp.front->next;
	while (temp.front != temp.rear)
	{
		cout << temp.front->data << "->";
		temp.front = temp.front->next;
	}
	cout << temp.front->data << endl;
	return OK;
}
int heapcs(duijiedian *heap)
{
	for (int i = 0; i <= MAX_AREA_NUM; i++)
	{
		heap[i].duimianji = 0;
		heap[i].duishenfen = i;
	}
	return OK;
}
int Maxscheng(duijiedian *heap, int specific, unsigned int mianjixu)
{
	unsigned int issue = heap[specific].duimianji;
	unsigned int index = heap[specific].duishenfen;
	unsigned int j = 2 * specific;
	while (j <= mianjixu)
	{
		if (j < mianjixu && heap[j].duimianji < heap[j + 1].duimianji)
			j = j + 1;
		if (issue >= heap[j].duimianji)
			break;
		heap[j / 2] = heap[j];
		j *= 2;
	}
	heap[j / 2].duimianji = issue;
	heap[j / 2].duishenfen = index;
	return OK;
}
int Minscheng(duijiedian *heap, int specific, unsigned int mianjixu)
{
	unsigned int issue = heap[specific].duimianji;
	unsigned int index = heap[specific].duishenfen;
	unsigned int j = 2 * specific;
	while (j <= mianjixu)
	{
		if (j < mianjixu && heap[j].duimianji > heap[j + 1].duimianji)
			j = j + 1;
		if (issue <= heap[j].duimianji)
			break;
		heap[j / 2] = heap[j];
		j *= 2;
	}
	heap[j / 2].duimianji = issue;
	heap[j / 2].duishenfen = index;
	return OK;
}
int dagendui(duijiedian *heap, unsigned int mianjixu)
{
	for (int i = mianjixu / 2; i >= 1; i--)
		Maxscheng(heap, i, mianjixu);
	return OK;
}
int xiaogendui(duijiedian *heap, unsigned int mianjixu)
{
	for (int i = mianjixu / 2; i >= 1; i--)
		Minscheng(heap, i, mianjixu);
	return OK;
}
int sortlarge(duijiedian *heap, unsigned int mianjixu)
{
	dagendui(heap, mianjixu);
	for (int i = mianjixu; i >= 2; i--)
	{
		duijiedian temp = heap[1];
		heap[1] = heap[i];
		heap[i] = temp;
		Maxscheng(heap, 1, i - 1);
	}
	return OK;
}
int sortdes(duijiedian *heap, unsigned int mianjixu)
{
	xiaogendui(heap, mianjixu);
	for (int i = mianjixu; i >= 2; i--)
	{
		duijiedian temp = heap[1];
		heap[1] = heap[i];
		heap[i] = temp;
		Minscheng(heap, 1, i - 1);
	}
	return OK;
}
//输出查找结果
int hlight1(duijiedian *heap, unsigned int inindex, unsigned int suind, Mat mask1, Mat gaoguang1, ajaclist AL)
{
	Mat src2 = imread("D:\\default.jpeg"), gray1, gaoguang1_copy;
	int i, j, k, index, is_target = FALSE;
	char str[10];
	cvtColor(src2, gray1, COLOR_BGR2GRAY);
	cvtColor(gray1, gray1, COLOR_GRAY2BGR);
	gaoguang1.copyTo(gaoguang1_copy);
	//查询区域面积是否在范围内，在则涂白，不在则涂黑（gaoguang1_copy）
	for (i = 0; i < mask1.rows; i++)
		for (j = 0; j < mask1.cols; j++)
		{
			index = mask1.at<int>(i, j);
			if (index > 0)
			{
				is_target = FALSE;
				for (k = inindex; k <= suind; k++)
				{
					if (index == heap[k].duishenfen)
					{
						is_target = TRUE;
						break;
					}
				}
				if (!is_target)
					gaoguang1_copy.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
			}
			else if (index == -1)
				gaoguang1_copy.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
		}
	gaoguang1_copy = gaoguang1_copy * 0.5 + gray1 * 0.5;
	//输出区域编号
	for (i = inindex; i <= suind; i++)
	{
		sprintf_s(str, "%d", heap[i].duishenfen);
		putText(gaoguang1_copy, str, AL.versy[heap[i].duishenfen - 1].gross, FONT_HERSHEY_PLAIN, 0.8, Scalar(255, 255, 255));
	}
	imshow("zheban search", gaoguang1_copy);
	return OK;
}
//折半查找
int zhebansearch(duijiedian *heap, int count, Mat mask1, Mat gaoguang1, ajaclist AL)
{
	unsigned int sup, inf, inindex = 0 /*存放下界下标*/, low = 1, high = count, mid, suind = 0 /*存放上界下标*/, i;
	int temp;
	//输入
	while (1)
	{
		cout << "the area range:" << heap[1].duimianji << "~" << heap[count].duimianji << endl;
		cout << "lower bound(-1 for quit):";
		cin >> temp;
		if (temp == -1)
			return EXIT;
		else if (temp < (int)heap[1].duimianji || temp >(int)heap[count].duimianji)
		{
			cout << "the lower bound must >=" << heap[1].duimianji << " and <=" << heap[count].duimianji << endl;
			continue;
		}
		inf = (unsigned int)temp;
		cout << "upper bound(-1 for quit):";
		cin >> temp;
		if (temp == -1)
			return EXIT;
		else if (temp <= (int)inf || temp > (int)heap[count].duimianji)
		{
			cout << "the upper bound must >=" << inf << " and <=" << heap[count].duimianji << endl;
			continue;
		}
		sup = (unsigned int)temp;
		break;
	}
	//查找下界
	while (low <= high)
	{
		mid = (low + high) / 2;
		if (heap[mid].duimianji == inf)
		{
			inindex = mid;
			break;
		}
		else if (heap[mid].duimianji < inf)
		{
			low = mid + 1;
			inindex = low;
		}
		else
			high = mid - 1;
	}
	//查找上界
	high = count;
	while (low <= high)
	{
		mid = (low + high) / 2;
		if (heap[mid].duimianji == sup)
		{
			suind = mid;
			break;
		}
		else if (heap[mid].duimianji < sup)
			low = mid + 1;
		else
		{
			high = mid - 1;
			suind = high;
		}
	}
	if (inindex > suind)
		cout << "No region area between" << inf << "~" << sup << "." << endl;
	else if (inindex <= suind)
	{
		cout << "regions the area between" << inf << "~" << sup << "are:" << endl;
		for (i = inindex; i <= suind; i++)
			cout << heap[i].duishenfen << ": " << heap[i].duimianji << endl;
		cout << (suind - inindex + 1) << " areas in total." << endl;
	}
	hlight1(heap, inindex, suind, mask1, gaoguang1, AL);
	cout << "\nClick [MISSION 1 RESULT] for search again." << endl;
	waitKey(0);
	return OK;
}
