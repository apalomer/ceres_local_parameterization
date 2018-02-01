#include "point_to_plane_functor.h"

PointToPlaneFunctor::PointToPlaneFunctor(double x, double y, double z)
{
    m_point= Point<double>(x,y,z);
}
PointToPlaneFunctor::PointToPlaneFunctor(Point<double> point)
{
    m_point= point;
}
