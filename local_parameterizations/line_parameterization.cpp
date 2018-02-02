
// Eigen
#include <Eigen/Dense>

// Local
#include "line_parameterization.h"
#include "tools/point.h"
#include "tools/line.h"

LineParameterization::LineParameterization()
{
}

LineParameterization::~LineParameterization() {}

bool LineParameterization::Plus(const double* x,const double* delta,double* x_plus_delta) const {

    // Get closest point to origin
    Line<double> line(x);
    double r,roll,pitch,yaw;
    line.to4parameters(r,roll,pitch,yaw);

    // Apply increments
    line.from4parameters(r+delta[0],roll+delta[1],pitch+delta[2],yaw+delta[3]);

    // Set to x_plus_delta
    Point<double> vector = line.vector();
    Point<double> point = line.point();
    x_plus_delta[Line<double>::V_X] = vector.x();
    x_plus_delta[Line<double>::V_Y] = vector.y();
    x_plus_delta[Line<double>::V_Z] = vector.z();
    x_plus_delta[Line<double>::P_X] = point.x();
    x_plus_delta[Line<double>::P_Y] = point.y();
    x_plus_delta[Line<double>::P_Z] = point.z();

    // Exit
    return true;
}

bool LineParameterization::ComputeJacobian(const double* x, double* jacobian) const{
    for (int i = 0;i<18;i++)jacobian[i] = 0;
    return true;
}

int LineParameterization::GlobalSize() const
{
    return 6;
}
int LineParameterization::LocalSize() const
{
    return 4;
}
