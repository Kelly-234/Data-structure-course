#include "Heap.hpp"

int HeapInit(HeapNode *heap)
{
    for (int i = 0; i <= MAX_RIGION_NUM; i++)
    {
        heap[i].area = 0;
        heap[i].id = i;
    }
    return OK;
}
void CaluHeapArea(Mat imgMask, HeapNode *heap)
{
    for (int i = 1; i < imgMask.rows - 1; i++)
    {
        int *rowInImgMask = imgMask.ptr<int>(i);
        for (int j = 1; j < imgMask.cols - 1; j++)
        {
            if (rowInImgMask[j] != -1 && rowInImgMask[j] > 0)
                heap[rowInImgMask[j]].area++;
        }
    }
}
void MaximizeHeapOnce(HeapNode *heap, int specific, unsigned int heapSortLimit)
{
    unsigned int issue = heap[specific].area;
    unsigned int index = heap[specific].id;
    unsigned int j = 2 * specific;
    while (j <= heapSortLimit)
    {
        if (j < heapSortLimit && heap[j].area < heap[j + 1].area)
            j = j + 1;
        if (issue >= heap[j].area)
            break;
        heap[j / 2] = heap[j];
        j *= 2;
    }
    heap[j / 2].area = issue;
    heap[j / 2].id = index;
}
void MaximizeHeap(HeapNode *heap, unsigned int heapSortLimit)
{
    for (int i = heapSortLimit / 2; i >= 1; i--)
        MaximizeHeapOnce(heap, i, heapSortLimit);
}
void HeapSortAsc(HeapNode *heap, unsigned int heapSortLimit)
{
    MaximizeHeap(heap, heapSortLimit);
    for (int i = heapSortLimit; i >= 2; i--)
    {
        HeapNode temp = heap[1];
        heap[1] = heap[i];
        heap[i] = temp;
        MaximizeHeapOnce(heap, 1, i - 1);
    }
}