#include "plane_parameterization.h"

PlaneParameterization::PlaneParameterization()
{
}
PlaneParameterization::~PlaneParameterization() {}

bool PlaneParameterization::Plus(const double* x,const double* delta,double* x_plus_delta) const
{
    ceres::HomogeneousVectorParameterization hvp(3);
    hvp.Plus(x,delta,x_plus_delta);
    x_plus_delta[3] = x[3] + delta[2];
    if (x_plus_delta[3] < 0 )
    {
        for (int i = 0; i < 4; i++)
            x_plus_delta[i] *= -1;
    }
    return true;
}

bool PlaneParameterization::ComputeJacobian(const double* x, double* jacobian) const
{
    ceres::HomogeneousVectorParameterization hvp(3);
    double hvpj[6];
    hvp.ComputeJacobian(x,hvpj);
    for (int i = 0;i<12;i++)jacobian[i] = 0;
    jacobian[0] = hvpj[0];
    jacobian[1] = hvpj[1];
    jacobian[3] = hvpj[2];
    jacobian[4] = hvpj[3];
    jacobian[6] = hvpj[4];
    jacobian[7] = hvpj[5];
    jacobian[11] = 1;
    return true;
}

int PlaneParameterization::GlobalSize() const
{
    return 4;
}
int PlaneParameterization::LocalSize() const
{
    return 3;
}
