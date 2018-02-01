#ifndef POLYNOMIALCOSTFUNTION_H
#define POLYNOMIALCOSTFUNTION_H

// Ceres
#include <ceres/sized_cost_function.h>

// Local
#include "tools/point2d.h"
#include "tools/polynomial.h"

// Dist cost funton from a Polynomial to a 2D point
class PolynomialCostFunction : public ceres::SizedCostFunction<2, 1>
{
public:

    PolynomialCostFunction(Polynomial<double> polynomial, Point2d<double> point);
    ~PolynomialCostFunction();

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;
private:
    Point2d<double> m_point;
    Polynomial<double> m_polynomial;
};
#endif // POLYNOMIALCOSTFUNTION_H
