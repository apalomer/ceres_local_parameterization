#ifndef PLANE_PARAMETERIZATION_H
#define PLANE_PARAMETERIZATION_H

// Ceres
#include <ceres/ceres.h>

class PlaneParameterization: public ceres::LocalParameterization {
public:
    explicit PlaneParameterization();
    virtual ~PlaneParameterization();

    virtual bool Plus(const double* x,const double* delta,double* x_plus_delta) const;

    virtual bool ComputeJacobian(const double* x, double* jacobian) const;

    virtual int GlobalSize() const;
    virtual int LocalSize() const;
};

#endif // PLANE_PARAMETERIZATION_H
