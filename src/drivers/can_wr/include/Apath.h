#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <stdbool.h>

#ifndef APATH_H
    #define APATH_H
#endif

#define TURE 1
#define FAULT 0

//约定：0是可走的，1表示障碍物不可走，2表示起点，3表示终点，4表示路径
#define int_0 0
#define int_1 1
#define int_2 2
#define int_3 3
#define int_4 4

#define MAP_MAX_X 10   //地图边界，二维数组大小
#define MAP_MAX_Y 10

typedef struct LNode {
    int data;    //对应数组中的数值
    int F;   //F = G + H;
    int G;   //G：从起点 A 移动到指定方格的移动代价，沿着到达该方格而生成的路径
    int H;   //H：从指定的方格移动到终点 B 的估算成本
    int x, y;   //对应数组中的坐标
    bool OPen_flag;  //在开放列表中为1，不在为0
    bool Close_flag;  //在关闭列表中为1，不在为0
    struct LNode* next;                    //用于链表排序
    struct LNode* path_next;            //用于最终找到的路径
}LNode, *LinkList;


void printfnode(LNode* curLNode);
LinkList InitList();  //返回一个初始化的链表
LNode** malloc_array2D(int row, int col);
void free_array2D(LNode **arr);
LNode** Translate_array(int array[10][10], int row, int col);    //将一个普通数组翻译为单链表节点的数组
void output(LNode **array, int row, int col);

LNode* find_start_LNode(LNode** Arr, int row, int col);    //从数组中找到始点
LNode* find_end_LNode(LNode** Arr, int row, int col);        //从数组中找到终点

//忘记这些要干嘛了，重写吧
bool isExist_ALNode_in_List(LNode* curLNode, LinkList L_OpenList);    //查看节点是否在链表中，在返回ture,不在返回fault
//对关闭列表中的当前节点进行检查，看它周围的节点是否在OpenList链表里，不在:添加进去；在：检查经过它到达起点的G是否最小，是：修改，不是：不修改
//LNode* check_CloseList_curLNode(LNode* curLNode, LNode* endLNode, LinkList L_OpenList, LinkList L_CloseList, LNode** Arr);   

LNode* pop_OpenList_minNode(LinkList L_OpenList);        //返回开放列表中F值最小的节点
void push_OpenList_Node(LinkList L, LNode *elem);   //插入一个节点并排序
bool insert_Into_CloseList(LNode* min_Open, LinkList L_CloseList);//插入OpenList中F值最小的节点到CloseList中去


int count_LNode_G(LNode* curLNode, LNode* aheadLNode);        //计算节点的G值
int count_LNode_H(LNode* curLNode, LNode* endLNode);        //计算节点的H值
int count_LNode_F(LNode* curLNode);        //计算节点的F值

bool isExist_openList(LNode* curLNode);    //查看节点是否在链表中，在返回ture,不在返回fault
bool isExist_closeList(LNode* curLNode);
bool isobstacle(LNode* curLNode);
void check_around_curNode(LNode* cur, LNode* endLNode, LinkList open_list, LNode** Arr);        //检查周围的节点，是否合适加入开放列表
