#include "polynomialcostfuntion.h"

PolynomialCostFunction::PolynomialCostFunction(Polynomial<double> polynomial, Point2d<double> point):
    m_point(point), m_polynomial(polynomial)
{

}
PolynomialCostFunction::~PolynomialCostFunction()
{

}

bool PolynomialCostFunction::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
{

    // Error
    Point2d<double> error = m_point - Point2d<double>(parameters[0][0],m_polynomial(parameters[0][0]));
    error.copyTo(residuals);

    // Compute the Jacobian if asked for.
    if (jacobians != NULL && jacobians[0] != NULL) {
        jacobians[0][0] = -1;
        jacobians[0][1] = -m_polynomial.jacobian(parameters[0][0]);
    }
    return true;
}
