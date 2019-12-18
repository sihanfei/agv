#include "utils.h"

namespace pnc
{

Box2d::Box2d(Vect corner1, Vect corner2, Vect corner3, Vect corner4)
{
  vertexes[0] = corner1;
  vertexes[1] = corner2;
  vertexes[2] = corner3;
  vertexes[3] = corner4;

  vertexCount = 4;

  for (uint32_t i = 0; i < vertexCount; ++i)
  {
    max_x = fmax(vertexes[i].x, max_x);
    min_x = fmin(vertexes[i].x, min_x);
    max_y = fmax(vertexes[i].y, max_y);
    min_y = fmin(vertexes[i].y, min_y);
  }
}

void Box2d::projectBox2d(Vect &axis, Box2d &box2d, double *min, double *max)
{
  double d = calSqr(axis, box2d.vertexes[0]);
  *min     = d;
  *max     = d;

  for (uint32_t i = 1; i < box2d.vertexCount; i++)
  {
    d    = calSqr(axis, box2d.vertexes[i]);
    *min = fmin(*min, d);
    *max = fmax(*max, d);
  }
}

Vect Box2d::calAxis(Vect &vect1, Vect &vect2)
{
  return Vect(vect1.x - vect2.x, vect1.y - vect2.y);
}

double Box2d::calSqr(Vect &vect1, Vect &vect2)
{
  return vect1.x * vect2.x + vect1.y * vect2.y;
}

bool Box2d::HasOverlap(Box2d &box2d)
{
  // 函数功能：判断两矩形是否相交，相交返回true，不相交返回false

  // 第一步：判断矩形的最大包络是否相交，若最大包络不相交，则两矩形不碰撞，否则，进行下一步判断
  if (box2d.max_x < min_x || box2d.min_x > max_x || box2d.max_y < min_y || box2d.min_y > max_y)
    return false;

  Vect axis, edge;
  double minA, maxA, minB, maxB;

  // 第二步：计算四条分离轴矢量（前提：矩形的四个顶点是依次产生的）
  for (uint32_t i = 0, j = vertexCount - 1; i < vertexCount + box2d.vertexCount; j = i, ++i)
  {
    if (i == 0)
    {
      edge = calAxis(vertexes[i], vertexes[j]);
      axis = edge;
    }
    else if (i < vertexCount)
    {
      axis = calAxis(vertexes[i], vertexes[j]);

      if (calSqr(edge, axis) < EPSILON)
      {
        i = vertexCount - 1;
      }
    }
    else if (i == vertexCount)
    {
      j    = vertexCount + box2d.vertexCount - 1;
      edge = calAxis(box2d.vertexes[i - vertexCount], box2d.vertexes[j - vertexCount]);
      axis = edge;
    }
    else
    {
      axis = calAxis(box2d.vertexes[i - vertexCount], box2d.vertexes[j - vertexCount]);
      if (calSqr(edge, axis) < EPSILON)
      {
        i = vertexCount + box2d.vertexCount - 1;
      }
    }

    // 第三步：计算顶点矢量在分离轴上的标量并选出最大最小标量
    projectBox2d(axis, *this, &minA, &maxA);
    projectBox2d(axis, box2d, &minB, &maxB);

    // 第四步：投影标量区间不重叠，则两矩形不想交，结束判断
    if (maxA < minB || maxB < minA)
    {
      return false;
    }
  }

  return true;
}
} // end namespace

