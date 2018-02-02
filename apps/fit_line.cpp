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

#define V_X 0.5
#define V_Y 0.5
#define V_Z 0.5
#define P_X 0
#define P_Y 0
#define P_Z 0
#define N_POINTS 1000
#define USE_LINE_PARAMETERIZATION false
#define VIEWER false

void usage(char** argv)
{
    std::cout<<"Usage: "<<argv[0]<<" OPTIONS\n\n";
    std::cout<<"OPTIONS:\n";
    std::cout<<"  -h    display this message\n";
    std::cout<<"  -l    line parameters for point and vector: v_x v_y v_z p_x p_y p_z (default: "<<V_X<<" "<<V_Y<<" "<<V_Z<<" "<<P_X<<" "<<P_Y<<" "<<P_Z<<")\n";
    std::cout<<"  -n    number of points to fit the line (default: "<<N_POINTS<<")\n";
    std::cout<<"  -p    use line parameterization (default: "<<(USE_LINE_PARAMETERIZATION?"true":"false")<<")\n";
#ifdef COMPILE_PCL
    std::cout<<"  -v    display vith 3d viewer (default: "<<(VIEWER?"true":"false")<<")\n";
#endif
    std::cout<<std::endl;
}

int main(int argc, char** argv)
{
    // Initialize logger
    google::InitGoogleLogging(argv[0]);

    // Line Gt
    Line<double> line_gt(V_X,V_Y,V_Z,P_X,P_Y,P_Z);

    // Number of samples
    int n_points(N_POINTS);

    // Use local parameterization
    bool use_line_parameterization(USE_LINE_PARAMETERIZATION);

#ifdef COMPILE_PCL
    // Viewer
    bool display(VIEWER);
#endif

    // Parse arguments
    int i = 1;
    while(i<argc)
    {
        std::string argument = argv[i];
        if (argument == "-h")
        {
            usage(argv);
            return 0;
        }
        else if (argument == "-l")
        {
            if (i+6 >= argc)
            {
                std::cout<<"Error: -l falg has to be followed by the line. Plese see help.\n";
                usage(argv);
                return -1;
            }
            double v_x, v_y, v_z;
            double p_x, p_y, p_z;
            v_x = atof(argv[++i]);
            v_y = atof(argv[++i]);
            v_z = atof(argv[++i]);
            p_x = atof(argv[++i]);
            p_y = atof(argv[++i]);
            p_z = atof(argv[++i]);
            line_gt.setVector(v_x,v_y,v_z);
            line_gt.setPoint(p_x,p_y,p_z);
        }
        else if (argument == "-n")
        {
            if (++i >= argc)
            {
                std::cout<<"Error: -n falg has to be followed by number of points to sample. Plese see help.\n";
                usage(argv);
                return -1;
            }
            else
                n_points = atoi(argv[i]);
        }
        else if (argument == "-p")
            use_line_parameterization = !use_line_parameterization;
#ifdef COMPILE_PCL
        else if (argument == "-v")
            display = !display;
#endif
        else
        {
            std::cout<<"Unrecognized option: "<<argument<<". Please see the help.\n";
            usage(argv);
            return 0;
        }
        i++;
    }

    // Display base line
    std::cout<<"Line gt: "<<line_gt<<std::endl;
    std::cout<<"Sampling "<<n_points<<" on line."<<std::endl;
    std::cout<<"Use local parameterization?"<<(use_line_parameterization?"true":"false")<<std::endl;

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
    LineFitIteratorViewer* lfiv;
    if (display)
        lfiv = new LineFitIteratorViewer(problem,line_gt,false);
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
        if (display)
            lfiv->addCostFunction(functor);
#endif
        ceres::CostFunction* cf = new ceres::AutoDiffCostFunction<PointToLineFunctor,3,6>(functor);
        problem->AddResidualBlock(cf,NULL,line.data());
    }

    // Set options
    if (use_line_parameterization)
        problem->SetParameterization(line.data(),new LineParameterization);
    ceres::Solver::Options options;
#ifdef COMPILE_PCL
    if (display)
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
    if (display)
        lfiv->viewer()->spin();
#endif

    // Exit
    delete problem;
    delete summary;
    return 0;
}
