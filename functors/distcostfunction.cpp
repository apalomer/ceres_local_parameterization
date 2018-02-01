#include "distcostfunction.h"


DistCostFunction::DistCostFunction(Point2d<double> pt):
    m_point(pt)
{

}
DistCostFunction::~DistCostFunction()
{

}

bool DistCostFunction::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {

    // Compute error
    Point2d<double> error = m_point - Point2d<double>(parameters[0][0],parameters[0][1]);
    error.copyTo(residuals);

    // Compute the Jacobian if asked for.
    if (jacobians != NULL && jacobians[0] != NULL) {
        jacobians[0][0] = -1;
        jacobians[0][1] = 0;
        jacobians[0][2] = 0;
        jacobians[0][3] = -1;
    }
    return true;
}
