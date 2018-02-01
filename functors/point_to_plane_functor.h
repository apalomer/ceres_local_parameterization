#ifndef POINT_TO_PLANE_FUNCTOR_H
#define POINT_TO_PLANE_FUNCTOR_H

// Local
#include "tools/point.h"
#include "tools/plane.h"

class PointToPlaneFunctor
{
public:
    PointToPlaneFunctor(double x, double y, double z);
    PointToPlaneFunctor(Point<double> point);

    template<typename T>
    bool operator()(T const* plane, T* residual) const
    {
        Plane<T> pl(plane);
        Point<T> point(T(m_point.x()),T(m_point.y()),T(m_point.z()));
        residual[0] = pl.distance(point);
        return true;
    }

private:
    Point<double> m_point;
};

#endif // POINT_TO_PLANE_FUNCTOR_H
