#ifndef LINE_PARAMETERIZATION_H
#define LINE_PARAMETERIZATION_H

// Ceres
#include <ceres/ceres.h>

// Local parametrization
class LineParameterization: public ceres::LocalParameterization {
public:
    explicit LineParameterization();
    virtual ~LineParameterization();

    virtual bool Plus(const double* x,const double* delta,double* x_plus_delta) const;

    virtual bool ComputeJacobian(const double* x, double* jacobian) const;

    virtual int GlobalSize() const;
    virtual int LocalSize() const;
};

#endif // LINE_PARAMETERIZATION_H
