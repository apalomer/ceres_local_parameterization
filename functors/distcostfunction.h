#ifndef DISTCOSTFUNCTION_H
#define DISTCOSTFUNCTION_H

// Ceres
#include <ceres/sized_cost_function.h>

// Local
#include "tools/point2d.h"

// Dist cost funton from a 2D point to a 2D point
class DistCostFunction : public ceres::SizedCostFunction<2, 2> {
public:

    DistCostFunction(Point2d<double> pt);
    ~DistCostFunction();

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;
private:
    Point2d<double> m_point;
};

#endif // DISTCOSTFUNCTION_H
