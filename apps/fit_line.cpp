// STD library
#include <iostream>
#include <sstream>
#include <stdexcept>

// Ceres
#include <ceres/ceres.h>

// Local
#include "tools/functions.h"
#include "tools/point.h"
#include "tools/line.h"
#include "functors/point_to_line_functor.h"
#include "local_parameterizations/line_parameterization.h"
#ifdef COMPILE_PCL
#include "iterator_viewers/line_fit_iterator_viewer.h"
#endif

int main(int argc, char** argv)
{
    // Initialize logger
    google::InitGoogleLogging(argv[0]);

    // Line Gt
    Line<double> line_gt(0.5,0.5,0.5,0,0,0);
    if (argc > 6)
        line_gt.set(atof(argv[1]),atof(argv[2]),atof(argv[3]),atof(argv[4]),atof(argv[5]),atof(argv[6]));
    std::cout<<"Line gt: "<<line_gt<<std::endl;

    // Number of samples
    int n_points(100);
    if (argc > 7)
        n_points = atoi(argv[7]);

    // Create problem
    ceres::Problem* problem = new ceres::Problem;

    // Set initial
    Point<double> line_vector(1,1,1);
    Point<double> line_point(0,0,0);
    Line<double> line(line_vector,line_point);
    Line<double> line_ini(line);

    // Create data
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0,0.1);
#ifdef COMPILE_PCL
    LineFitIteratorViewer* lfiv = new LineFitIteratorViewer(problem,line_gt,false);
#endif
    for (int i = 0; i < n_points; i++)
    {

        // Create point
        double t = fRand(-100,100);
        Point<double> point = line_gt(t);
        point.setZ(point.z()+distribution(generator));

        // Cost function
        PointToLineFunctor* functor = new PointToLineFunctor(point);
#ifdef COMPILE_PCL
        lfiv->addCostFunction(functor);
#endif
        ceres::CostFunction* cf = new ceres::AutoDiffCostFunction<PointToLineFunctor,1,6>(functor);
        problem->AddResidualBlock(cf,NULL,line.data());
    }

    // Set options
    problem->SetParameterization(line.data(),new LineParameterization);
    ceres::Solver::Options options;
#ifdef COMPILE_PCL
    options.callbacks.push_back(lfiv);
#endif
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.update_state_every_iteration = true;
    options.max_num_iterations = 10000;

    // Solve
    ceres::Solver::Summary* summary = new ceres::Solver::Summary;
    ceres::Solve(options, problem, summary);

    // Display result
    std::cout<<summary->FullReport()<<std::endl;
    std::cout<<" Line: "<<line_ini<<" -> "<<line<<std::endl;
    std::cout<<" Gt:   "<<line_gt<<std::endl;
#ifdef COMPILE_PCL
    lfiv->viewer()->spin();
#endif

    // Exit
    delete problem;
    delete summary;
    return 0;
}
