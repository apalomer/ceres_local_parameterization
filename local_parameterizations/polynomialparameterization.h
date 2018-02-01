#ifndef POLYNOMIALPARAMETERIZATION_H
#define POLYNOMIALPARAMETERIZATION_H

// Ceres
#include <ceres/local_parameterization.h>

// Local
#include "tools/polynomial.h"

// Local parametrization
class PolynomialParameterization: public ceres::LocalParameterization {
public:
    explicit PolynomialParameterization(Polynomial<double> polynomial);
    virtual ~PolynomialParameterization();

    virtual bool Plus(const double* x,const double* delta,double* x_plus_delta) const;

    virtual bool ComputeJacobian(const double* x, double* jacobian) const;

    virtual int GlobalSize() const;
    virtual int LocalSize() const;

private:
    Polynomial<double> m_polynomial;
};

#endif // POLYNOMIALPARAMETERIZATION_H
