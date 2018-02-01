#include "polynomialparameterization.h"

PolynomialParameterization::PolynomialParameterization(Polynomial<double> polynomial):
    m_polynomial(polynomial)
{
}
PolynomialParameterization::~PolynomialParameterization()
{

}

bool PolynomialParameterization::Plus(const double* x,const double* delta,double* x_plus_delta) const
{
    x_plus_delta[0] = delta[0] + x[0];
    x_plus_delta[1] = m_polynomial(x_plus_delta[0]);
    return true;
}

bool PolynomialParameterization::ComputeJacobian(const double* x, double* jacobian) const
{
    jacobian[0] = 1;
    jacobian[1] = m_polynomial.jacobian(x[0]);
    return true;
}

int PolynomialParameterization::GlobalSize() const
{
    return 2;
}
int PolynomialParameterization::LocalSize() const
{
    return 1;
}
