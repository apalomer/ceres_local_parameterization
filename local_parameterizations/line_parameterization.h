#ifndef LINE_PARAMETERIZATION_H
#define LINE_PARAMETERIZATION_H

// Ceres
#include <ceres/ceres.h>

// Local
#include "tools/line.h"
#include "tools/point.h"

struct LineParameterizationPlus
{
    template<typename T>
    bool operator()(const T* x, const T* delta, T* x_plus_delta) const
    {

        // Get closest point to origin
        Line<T> line(x);
        T r,roll,pitch,yaw;
        line.to4parameters(r,roll,pitch,yaw);

        // Apply increments
        line.from4parameters(r+delta[0],roll+delta[1],pitch+delta[2],yaw+delta[3]);

        // Set to x_plus_delta
        Point<T> vector = line.vector();
        Point<T> point = line.point();
        x_plus_delta[Line<T>::V_X] = vector.x();
        x_plus_delta[Line<T>::V_Y] = vector.y();
        x_plus_delta[Line<T>::V_Z] = vector.z();
        x_plus_delta[Line<T>::P_X] = point.x();
        x_plus_delta[Line<T>::P_Y] = point.y();
        x_plus_delta[Line<T>::P_Z] = point.z();

        // Exit
        return true;
    }
};

// Local parametrization
typedef ceres::AutoDiffLocalParameterization<LineParameterizationPlus, 6, 4> LineParameterization;

#endif // LINE_PARAMETERIZATION_H
