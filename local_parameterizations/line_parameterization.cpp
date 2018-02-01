#include "line_parameterization.h"


LineParameterization::LineParameterization()
{
}

LineParameterization::~LineParameterization() {}

bool LineParameterization::Plus(const double* x,const double* delta,double* x_plus_delta) const {

    // Line vector
    ceres::HomogeneousVectorParameterization hvp(3);
    hvp.Plus(x,delta,x_plus_delta);

    // Line point
    x_plus_delta[3] = x[3] + delta[2];
    x_plus_delta[4] = x[4] + delta[3];
    x_plus_delta[5] = x[5] + delta[4];
    return true;
}

bool LineParameterization::ComputeJacobian(const double* x, double* jacobian) const{
    ceres::HomogeneousVectorParameterization hvp(3);
    double hvpj[6];
    hvp.ComputeJacobian(x,hvpj);
    for (int i = 0;i<30;i++)jacobian[i] = 0;
    jacobian[0] = hvpj[0];
    jacobian[1] = hvpj[1];
    jacobian[5] = hvpj[2];
    jacobian[6] = hvpj[3];
    jacobian[10] = hvpj[4];
    jacobian[11] = hvpj[5];
    jacobian[17] = 1;
    jacobian[23] = 1;
    jacobian[29] = 1;
    return true;
}

int LineParameterization::GlobalSize() const
{
    return 6;
}
int LineParameterization::LocalSize() const
{
    return 5;
}
