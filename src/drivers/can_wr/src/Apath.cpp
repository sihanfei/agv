#include "Apath.h"
#include <ros/ros.h>

void printfnode(LNode* curLNode)
{
    ROS_INFO("data:%d",curLNode->data);
    ROS_INFO("F:%d",curLNode->F);
    ROS_INFO("G:%d",curLNode->G);
    ROS_INFO("H:%d",curLNode->H);
    ROS_INFO("x:%d,y:%d",curLNode->x,curLNode->y);
    ROS_INFO("OPen_flag:%d,Close_flag:%d",curLNode->OPen_flag,curLNode->Close_flag);
    ROS_INFO_STREAM("next:" << curLNode->next << ",path_next:" << curLNode->path_next);
    printf("************************************\n");
}

LinkList InitList()
{
    LinkList L = (LinkList)malloc(sizeof(LNode));
    if (L == NULL)
    {
        printf("Defeat!");
        exit(1);
    }
    memset(L,0,sizeof(LNode));
    return L;
}//LinkList()

LNode** malloc_array2D(int row, int col)
{
    LNode** map = (LNode**)malloc(row*sizeof(LNode*) + row*col*sizeof(LNode));
    LNode* head = (LNode*)(map + row);
    for (int i = 0; i < row; ++i)
        map[i] = head + i*col;
    return map;
}

LNode** Translate_array(int array[][10], int row, int col)
{
    LNode **map = malloc_array2D(10, 10);
    for (int i = 0; i < row; ++i)
        for (int j = 0; j < col; ++j)
        {
            (map[i] + j)->data = array[i][j];
            (map[i] + j)->G = 0;
            (map[i] + j)->H = 0;
            (map[i] + j)->F = 0;    //(map[i] + j)->G + (map[i] + j)->H;
            (map[i] + j)->x = i;
            (map[i] + j)->y = j;
            (map[i] + j)->Close_flag = 0;
            (map[i] + j)->OPen_flag = 0;
            (map[i] + j)->next = NULL;
            (map[i] + j)->path_next = NULL;
        }
    return map;
}//Translate_array()

void free_array2D(LNode **arr)
{
    free(arr);
}

void output(LNode** array, int row, int col)  //二维数组的访问必须指明位数，否则编译器不能解析
{
    //for (int i = 0; i < row; ++i)
    //    for (int j = 0; j < col; ++j)
    //    {
    //        (array[i] + j)->F = j;
    //    }
    for (int i = 0; i < row; ++i)
    {
        for (int j = 0; j < col; ++j)
        {
            printf("%d\t", (array[i] + j)->data);
        }
        printf("\n");
    }
}

LNode* find_start_LNode(LNode** Arr, int row, int col)    //从数组中找到始点
{
    LNode* start_LNode = NULL;
    for (int i = 0; i < row; ++i)
    {
        for (int j = 0; j < col; ++j)
        {
            if (2 == (Arr[i] + j)->data)
            {
                start_LNode = (Arr[i] + j);
                //起点H=0,G=0,F=0
                start_LNode->G = 0;
                start_LNode->H = 0;
                start_LNode->F = 0;        //起点，则默认所有值为0
                return start_LNode;        //返回节点
            }
        }
    }
    return NULL;
}
LNode* find_end_LNode(LNode** Arr, int row, int col)        //从数组中找到终点
{
    LNode* end_LNode = NULL;
    for (int i = 0; i < row; ++i)
    {
        for (int j = 0; j < col; ++j)
        {
            if (3 == (Arr[i] + j)->data)
            {
                end_LNode = (*(Arr + i) + j);
                end_LNode->F = 0;
                end_LNode->G = 0;
                end_LNode->H = 0;
                return end_LNode;        //返回节点
            }
        }
    }
    return NULL;
}

int count_LNode_G(LNode* curLNode, LNode* aheadLNode)        //计算节点的G值
{
    if (curLNode->x == aheadLNode->y && curLNode->y == aheadLNode->y)
        return 0;
    if (aheadLNode->x - curLNode->x != 0 && aheadLNode->y - curLNode->y !=0)
        curLNode->G = aheadLNode->G + 14;
    else
        curLNode->G = aheadLNode->G + 10;
    return curLNode->G;
}
int count_LNode_H(LNode* curLNode, LNode* endLNode)        //计算节点的H值
{
    curLNode->H = abs(endLNode->x - curLNode->x) * 10 + abs(endLNode->y - curLNode->y) * 10;
    return curLNode->H;
}
int count_LNode_F(LNode* curLNode)        //计算节点的F值
{
    curLNode->F = curLNode->G + curLNode->H;
    return curLNode->F;
}

void push_OpenList_Node(LinkList L, LNode *elem)        //按从小到大的顺序
{
    LNode *p, *q;
    p = q = L;
    printfnode(L);
    printfnode(elem);
    // ROS_INFO_STREAM(p->x << "," << p->y  << "," << p->next << "," << p->F << "," << elem->F );
    while (p->next != NULL && p->F < elem->F)
    {
        printf("!\n");
        q = p;
        p = p->next;
    }
    if (p->F < elem->F) q = p;
    elem->next = q->next;
    q->next = elem;
    //插入成功，更改属性值OPen_flag = 1
    elem->OPen_flag = 1;
}
LNode* pop_OpenList_minNode(LinkList L_OpenList )        //返回开放列表中F值最小的节点
{
    LNode *elem = NULL;
    if (L_OpenList->next)        //为了安全，防止访问空指针
    {
        L_OpenList->next->OPen_flag = 0;
        elem = L_OpenList->next;
        L_OpenList->next = L_OpenList->next->next;
        elem->next = NULL;
    }
    else
        printf("have a NULL point in pop_OpenList_mimNode()");
    return elem;
}
bool insert_Into_CloseList(LNode* min_Open, LinkList L_CloseList)//插入OpenList中F值最小的节点到CloseList中去
{
    //对于CloseList中的节点并不需要排序,采用头插法
    min_Open->next = L_CloseList->next;
    L_CloseList->next = min_Open;
    min_Open->Close_flag = 1;
    return TURE;
}

bool isExist_openList(LNode* curLNode)
{
    return curLNode->OPen_flag;
}
bool isExist_closeList(LNode* curLNode)
{
    return curLNode->Close_flag;
}
bool isobstacle(LNode* curLNode)
{
    if (curLNode->data == 1)
        return TURE;
    else
        return FAULT;
}
bool isJoin(LNode* cur)        //该节点是否可以加入开放列表
{
    if (cur->x > -1 && cur->y > -1)            //边界检测
    {
        if (!isExist_closeList(cur) && !isobstacle(cur))        //既不在关闭列表里，也不是障碍物
        {
            return TURE;
        }
        else
            return FAULT;
    }
    return FAULT;
}
void insert_open(LNode *Node, LNode* ahead, LNode* endLNode, LinkList open_list, LNode** Arr)
{
    if (isJoin(Node))
    {
        if (isExist_openList(Node))
        {
            if (Node->x - ahead->x != 0 && Node->y - ahead->y != 0) {
                if (Node->F > (ahead->F + 14))
                {
                    count_LNode_G(Node, ahead);
                    count_LNode_F(Node);        //H值没有改变，所以还是原来的值
                    Node->path_next = ahead;        //也不用再插入
                }
            }
            else {
                if (Node->F > (ahead->F + 10))
                {
                    count_LNode_G(Node, ahead);
                    count_LNode_F(Node);        //H值没有改变，所以还是原来的值
                    Node->path_next = ahead;        //也不用再插入
                }
            }
        }
        else {
            count_LNode_G(Node, ahead);
            count_LNode_H(Node, endLNode);
            count_LNode_F(Node);
            Node->path_next = ahead;
            push_OpenList_Node(open_list, Node);
        }
    }
}
void check_around_curNode(LNode* cur, LNode* endLNode, LinkList open_list, LNode** Arr)
{
    int x = cur->x;
    int y = cur->y;
    insert_open(Arr[x] + y - 1, cur, endLNode, open_list, Arr);
    insert_open(Arr[x] + y + 1, cur, endLNode, open_list, Arr);
    insert_open(Arr[x + 1] + y, cur, endLNode, open_list, Arr);
    insert_open(Arr[x + 1] + y - 1, cur, endLNode, open_list, Arr);
    insert_open(Arr[x + 1] + y + 1, cur, endLNode, open_list, Arr);
    insert_open(Arr[x - 1] + y, cur, endLNode, open_list, Arr);
    insert_open(Arr[x - 1] + y + 1, cur, endLNode, open_list, Arr);
    insert_open(Arr[x - 1] + y - 1, cur, endLNode, open_list, Arr);
}
