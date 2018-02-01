// STD library
#include <iostream>

// Ceres
#include <ceres/ceres.h>

// Local
#include "tools/functions.h"
#include "tools/point.h"
#include "tools/plane.h"
#include "functors/point_to_plane_functor.h"
#include "local_parameterizations/plane_parameterization.h"

int main(int argc, char** argv)
{
    // Get ground truth plane
    Plane<double> plane_gt(0,0,-1,10);
    if (argc > 4)
        plane_gt.set(atof(argv[1]),atof(argv[2]),atof(argv[3]),atof(argv[4]));
    int n_points(1000);
    if (argc > 5)
        n_points = atoi(argv[5]);

    // Create problem
    ceres::Problem* problem = new ceres::Problem;

    // Set initial
    Plane<double> plane(1,0,0,100);
    Plane<double> plane_ini(plane);

    // Create data
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0,0.1);
    for (int i = 0; i < n_points; i++)
    {
        double x;
        double y;
        double z;
        if (plane_gt.normal().z() == 0)
        {
            y = fRand(-1000,1000);
            z = fRand(-1000,1000);
            x = (plane_gt.d() - plane_gt.normal().z()*z - plane_gt.normal().y()*y)/plane_gt.normal().x() + distribution(generator);
        }
        else
        {
            x = fRand(-1000,1000);
            y = fRand(-1000,1000);
            z = (plane_gt.d() - plane_gt.normal().x()*x - plane_gt.normal().y()*y)/plane_gt.normal().z() + distribution(generator);
        }
        Point<double> point(x,y,z);
        PointToPlaneFunctor* functor = new PointToPlaneFunctor(point);
        ceres::CostFunction* cf = new ceres::AutoDiffCostFunction<PointToPlaneFunctor,1,4>(functor);
        problem->AddResidualBlock(cf,NULL,plane.data());
    }


    // Set options
    problem->SetParameterization(plane.data(),new PlaneParameterization);
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // Solve
    ceres::Solver::Summary* summary = new ceres::Solver::Summary;
    ceres::Solve(options, problem, summary);

    // Display result
    std::cout<<summary->FullReport()<<std::endl;
    std::cout<<" plane: "<< plane_ini <<" -> "<< plane <<std::endl;
    std::cout<<" gt:    "<< plane_gt <<std::endl;
    std::cout<<" Error: "<< plane_ini - plane_gt << " -> " << plane - plane_gt <<std::endl;

    // Exit
    return 0;
}
