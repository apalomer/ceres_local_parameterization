#include "point_to_line_functor.h"

PointToLineFunctor::PointToLineFunctor(double x, double y, double z, bool debug):
    m_debug(debug)
{
    m_point.set(x,y,z);
}
PointToLineFunctor::PointToLineFunctor(Point<double> point, bool debug):
    m_debug(debug)
{
    m_point = point;
}

Point<double> PointToLineFunctor::point()
{
    return m_point;
}
