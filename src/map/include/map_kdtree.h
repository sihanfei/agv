#ifndef MAP_INCLUDE_GET_REF_LINE_H_
#define MAP_INCLUDE_GET_REF_LINE_H_

#include "map_common_utils.h"
#include <algorithm>
#include <iostream>
#include <math.h>
#include <stack>

using namespace std;

namespace superg_agv
{
namespace map
{
class RefPointKDTree
{
public:
  RefPointKDTree()
  {
  }
  virtual ~RefPointKDTree()
  {
  }

  int creakeKDTreeFromRefPointVec(const std::vector< RefPoints > &ref_points_vec_)
  {
  }

  bool cmp1(data a, data b)
  {
    return a.x < b.x;
  }

  bool cmp2(data a, data b)
  {
    return a.y < b.y;
  }

  bool equal(data a, data b)
  {
    if (a.x == b.x && a.y == b.y)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  void ChooseSplit(data exm_set[], int size, int &split, data &SplitChoice)
  {
    /*compute the variance on every dimension. Set split as the dismension that have the biggest
     variance. Then choose the instance which is the median on this split dimension.*/
    /*compute variance on the x,y dimension. DX=EX^2-(EX)^2*/
    double tmp1, tmp2;
    tmp1 = tmp2 = 0;
    for (int i = 0; i < size; ++i)
    {
      tmp1 += 1.0 / ( double )size * exm_set[i].x * exm_set[i].x;
      tmp2 += 1.0 / ( double )size * exm_set[i].x;
    }
    double v1 = tmp1 - tmp2 * tmp2; // compute variance on the x dimension

    tmp1 = tmp2 = 0;
    for (int i = 0; i < size; ++i)
    {
      tmp1 += 1.0 / ( double )size * exm_set[i].y * exm_set[i].y;
      tmp2 += 1.0 / ( double )size * exm_set[i].y;
    }
    double v2 = tmp1 - tmp2 * tmp2; // compute variance on the y dimension

    split = v1 > v2 ? 0 : 1; // set the split dimension

    if (split == 0)
    {
      sort(exm_set, exm_set + size, cmp1);
    }
    else
    {
      sort(exm_set, exm_set + size, cmp2);
    }

    // set the split point value
    SplitChoice.x = exm_set[size / 2].x;
    SplitChoice.y = exm_set[size / 2].y;
  }

  Tnode *build_kdtree(data exm_set[], int size, Tnode *T)
  {
    // call function ChooseSplit to choose the split dimension and split point
    if (size == 0)
    {
      return NULL;
    }
    else
    {
      int split;
      data dom_elt;
      ChooseSplit(exm_set, size, split, dom_elt);
      data exm_set_right[100];
      data exm_set_left[100];
      int sizeleft, sizeright;
      sizeleft = sizeright = 0;

      if (split == 0)
      {
        for (int i = 0; i < size; ++i)
        {

          if (!equal(exm_set[i], dom_elt) && exm_set[i].x <= dom_elt.x)
          {
            exm_set_left[sizeleft].x = exm_set[i].x;
            exm_set_left[sizeleft].y = exm_set[i].y;
            sizeleft++;
          }
          else if (!equal(exm_set[i], dom_elt) && exm_set[i].x > dom_elt.x)
          {
            exm_set_right[sizeright].x = exm_set[i].x;
            exm_set_right[sizeright].y = exm_set[i].y;
            sizeright++;
          }
        }
      }
      else
      {
        for (int i = 0; i < size; ++i)
        {

          if (!equal(exm_set[i], dom_elt) && exm_set[i].y <= dom_elt.y)
          {
            exm_set_left[sizeleft].x = exm_set[i].x;
            exm_set_left[sizeleft].y = exm_set[i].y;
            sizeleft++;
          }
          else if (!equal(exm_set[i], dom_elt) && exm_set[i].y > dom_elt.y)
          {
            exm_set_right[sizeright].x = exm_set[i].x;
            exm_set_right[sizeright].y = exm_set[i].y;
            sizeright++;
          }
        }
      }
      T            = new Tnode;
      T->dom_elt.x = dom_elt.x;
      T->dom_elt.y = dom_elt.y;
      T->split     = split;
      T->left      = build_kdtree(exm_set_left, sizeleft, T->left);
      T->right     = build_kdtree(exm_set_right, sizeright, T->right);
      return T;
    }
  }

  double Distance(data a, data b)
  {
    double tmp = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
    return sqrt(tmp);
  }

  void searchNearest(Tnode *Kd, data target, data &nearestpoint, double &distance)
  {
    // 1. 如果Kd是空的，则设dist为无穷大返回
    // 2. 向下搜索直到叶子结点
    stack< Tnode * > search_path;
    Tnode *pSearch = Kd;
    data nearest;
    double dist;

    while (pSearch != NULL)
    {
      // pSearch加入到search_path中;
      search_path.push(pSearch);

      if (pSearch->split == 0)
      {
        if (target.x <= pSearch->dom_elt.x) /* 如果小于就进入左子树 */
        {
          pSearch = pSearch->left;
        }
        else
        {
          pSearch = pSearch->right;
        }
      }
      else
      {
        if (target.y <= pSearch->dom_elt.y) /* 如果小于就进入左子树 */
        {
          pSearch = pSearch->left;
        }
        else
        {
          pSearch = pSearch->right;
        }
      }
    }
    //取出search_path最后一个赋给nearest
    nearest.x = search_path.top()->dom_elt.x;
    nearest.y = search_path.top()->dom_elt.y;
    search_path.pop();

    dist = Distance(nearest, target);
    // 3. 回溯搜索路径

    Tnode *pBack;

    while (search_path.size() != 0)
    {
      //取出search_path最后一个结点赋给pBack
      pBack = search_path.top();
      search_path.pop();

      if (pBack->left == NULL && pBack->right == NULL) /* 如果pBack为叶子结点 */

      {

        if (Distance(nearest, target) > Distance(pBack->dom_elt, target))
        {
          nearest = pBack->dom_elt;
          dist    = Distance(pBack->dom_elt, target);
        }
      }

      else

      {

        int s = pBack->split;
        if (s == 0)
        {
          if (fabs(pBack->dom_elt.x - target.x) <
              dist) /* 如果以target为中心的圆（球或超球），半径为dist的圆与分割超平面相交，
                       那么就要跳到另一边的子空间去搜索 */
          {
            if (Distance(nearest, target) > Distance(pBack->dom_elt, target))
            {
              nearest = pBack->dom_elt;
              dist    = Distance(pBack->dom_elt, target);
            }
            if (target.x <= pBack->dom_elt.x) /* 如果target位于pBack的左子空间，那么就要跳到右子空间去搜索 */
              pSearch = pBack->right;
            else
              pSearch = pBack->left; /* 如果target位于pBack的右子空间，那么就要跳到左子空间去搜索 */
            if (pSearch != NULL)
              // pSearch加入到search_path中
              search_path.push(pSearch);
          }
        }
        else
        {
          if (fabs(pBack->dom_elt.y - target.y) <
              dist) /* 如果以target为中心的圆（球或超球），半径为dist的圆与分割超平面相交，
                       那么就要跳到另一边的子空间去搜索 */
          {
            if (Distance(nearest, target) > Distance(pBack->dom_elt, target))
            {
              nearest = pBack->dom_elt;
              dist    = Distance(pBack->dom_elt, target);
            }
            if (target.y <= pBack->dom_elt.y) /* 如果target位于pBack的左子空间，那么就要跳到右子空间去搜索 */
              pSearch = pBack->right;
            else
              pSearch = pBack->left; /* 如果target位于pBack的右子空间，那么就要跳到左子空间去搜索 */
            if (pSearch != NULL)
              // pSearch加入到search_path中
              search_path.push(pSearch);
          }
        }
      }
    }

    nearestpoint.x = nearest.x;
    nearestpoint.y = nearest.y;
    distance       = dist;
  }

public:

struct RefPoints
{
  int ref_point_id;
  int ref_line_id;
  Point3 point;
  int line_count;
  std::vector< LineWidth > line_width;
  double kappa; // cuv：1/米
  double dappa; // gcuv：1/米^2
  double d_from_line_begin;
  double theta; // 弧度（0-2pi）
};


  struct data
  {
    double x = 0;
    double y = 0;
  };

  struct Tnode
  {
    struct data dom_elt;
    int split;
    struct Tnode *left;
    struct Tnode *right;
  };
};

} // namespace map
} // namespace superg_agv
#endif