
#ifndef BOX2D_H
#define BOX2D_H
#include "common/utils.h"

namespace planning
{

struct Vect
{
  double x, y;
  Vect()
  {
  }
  Vect(double Cx, double Cy)
  {
    x = Cx;
    y = Cy;
  }
};

class Box2d
{
public:
  Box2d(Vect corner1, Vect corner2, Vect corner3, Vect corner4);
  Box2d()  = default;
  ~Box2d() = default;

  bool HasOverlap(Box2d &box2d);

private:
  void projectBox2d(Vect &axis, Box2d &box2d, double *min, double *max);

  // double intervalDistance(double minA, double maxA, double minB, double maxB);

  Vect calAxis(Vect &vect1, Vect &vect2);
  double calSqr(Vect &vect1, Vect &vect2);

  double max_x = std::numeric_limits< double >::min();
  double min_x = std::numeric_limits< double >::max();
  double max_y = std::numeric_limits< double >::min();
  double min_y = std::numeric_limits< double >::max();

  int vertexCount;
  Vect vertexes[4];
};

} // end namespace
#endif
