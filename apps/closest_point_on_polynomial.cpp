// STD library
#include <iostream>

// Cres
#include <ceres/ceres.h>

// Qt
#ifdef COMPILE_QT
#include <qcustomplot.h>
#include <QApplication>
#endif

// Local
#include "tools/point2d.h"
#include "tools/polynomial.h"
#include "functors/distcostfunction.h"
#include "functors/polynomialcostfuntion.h"
#include "local_parameterizations/polynomialparameterization.h"

// Main
int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);

    // Check inputs
    if (argc<5)
    {
        std::cout<<"Usage: "<<argv[0]<<" point_x point_y x_0 x_1 x_2 ..."<<std::endl;
        return -1;
    }

    // Get Point
    Point2d<double> point(atof(argv[1]),atof(argv[2]));

    // Get polynomial
    Polynomial<double> polynomial;
    for ( int i = 3 ; i < argc ; i++ ) polynomial.push_back(atof(argv[i]));

    // Select point on polygon
    Point2d<double> p0(0,polynomial(0));
    Point2d<double> p(p0);

    // Using local parameterization
    DistCostFunction* dcf = new DistCostFunction(point);
    PolynomialParameterization* polynomial_parametrization = new PolynomialParameterization(polynomial);
    ceres::Problem problem;
    problem.AddResidualBlock(dcf,NULL,p.getPtr());
    problem.SetParameterization(p.getPtr(),polynomial_parametrization);
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.function_tolerance = 1e-10;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    // Usin PolynomialCostFunction;
    PolynomialCostFunction* pcf = new PolynomialCostFunction(polynomial,point);
    ceres::Problem problem2;
    Point2d<double> p20(p0.x(),polynomial(p0.x()));
    Point2d<double> p2(p20);
    problem2.AddResidualBlock(pcf,NULL,p2.getPtr());
    ceres::Solver::Options options2;
    options2.minimizer_progress_to_stdout = true;
    options2.function_tolerance = 1e-10;
    ceres::Solver::Summary summary2;
    ceres::Solve(options2, &problem2, &summary2);
    p2.setY(polynomial(p2.x()));
    std::cout << summary2.FullReport() << std::endl;

    // Display results
    std::cout << "f(x): " << polynomial << std::endl;
    std::cout << "p : " << point << std::endl;
    std::cout << "cp (LocalParameterization): " << p0 << " -> " << p << std::endl;
    std::cout << "cp (PolynomialCstFunction): " << p20 << " -> " << p2 << std::endl;

#ifdef COMPILE_QT
    // Create data
    QVector<double> x,y;
    for (double x_ = -1; x_ <= 2; x_ += 0.01)
    {
        x.push_back(x_);
        y.push_back(polynomial(x_));
    }
    // Dispaly graph
    /// Create Qapplication
    QApplication app(argc,argv);
    /// Cretate ui
    QCustomPlot* customPlot = new QCustomPlot();
    /// Change size
    customPlot->resize(800,600);
    /// create graph
    customPlot->addGraph();
    /// add data
    customPlot->graph(0)->setData(x, y);
    customPlot->graph(0)->setName("Polygon");
    customPlot->addGraph();
    customPlot->graph(1)->setData(QVector<double>(1,point.x()),QVector<double>(1,point.y()));
    customPlot->graph(1)->setPen(QPen(Qt::red));
    customPlot->graph(1)->setLineStyle(QCPGraph::lsNone);
    customPlot->graph(1)->setScatterStyle(QCPScatterStyle::ssStar);
    customPlot->graph(1)->setName("Point to find closest on polygon");
    /// add initial point
    customPlot->addGraph();
    customPlot->graph(2)->setData(QVector<double>(1,p0.x()),QVector<double>(1,p0.y()));
    customPlot->graph(2)->setPen(QPen(Qt::green));
    customPlot->graph(2)->setLineStyle(QCPGraph::lsNone);
    customPlot->graph(2)->setScatterStyle(QCPScatterStyle::ssStar);
    customPlot->graph(2)->setName("Initial point (LP)");
    /// add final point
    customPlot->addGraph();
    customPlot->graph(3)->setData(QVector<double>(1,p.x()),QVector<double>(1,p.y()));
    customPlot->graph(3)->setPen(QPen(Qt::yellow));
    customPlot->graph(3)->setLineStyle(QCPGraph::lsNone);
    customPlot->graph(3)->setScatterStyle(QCPScatterStyle::ssStar);
    customPlot->graph(3)->setName("Final point (LP)");
    /// add initial point
    customPlot->addGraph();
    customPlot->graph(4)->setData(QVector<double>(1,p20.x()),QVector<double>(1,p20.y()));
    customPlot->graph(4)->setPen(QPen(Qt::cyan));
    customPlot->graph(4)->setLineStyle(QCPGraph::lsNone);
    customPlot->graph(4)->setScatterStyle(QCPScatterStyle::ssStar);
    customPlot->graph(4)->setName("Initial point (QCF)");
    /// add final point
    customPlot->addGraph();
    customPlot->graph(5)->setData(QVector<double>(1,p2.x()),QVector<double>(1,p2.y()));
    customPlot->graph(5)->setPen(QPen(Qt::black));
    customPlot->graph(5)->setLineStyle(QCPGraph::lsNone);
    customPlot->graph(5)->setScatterStyle(QCPScatterStyle::ssStar);
    customPlot->graph(5)->setName("Final point (QCF)");
    /// Compute normal at solution
    Point2d<double> np2(polynomial.jacobian(p2.x()),-1);
    customPlot->addGraph();
    QVector<double> np2_x;
    np2_x.push_back(p2.x());
    np2_x.push_back(p2.x()+2*np2.x());
    QVector<double> np2_y;
    np2_y.push_back(p2.y());
    np2_y.push_back(p2.y() + 2*np2.y());
    customPlot->graph(6)->setData(np2_x,np2_y);
    customPlot->graph(6)->setPen(QPen(Qt::black));
    customPlot->graph(6)->setName("Normal point (QCF)");
    /// Compute normal at solution
    Point2d<double> np(polynomial.jacobian(p.x()),-1);
    customPlot->addGraph();
    QVector<double> np_x;
    np_x.push_back(p.x());
    np_x.push_back(p.x()+2*np.x());
    QVector<double> np_y;
    np_y.push_back(p.y());
    np_y.push_back(p.y() + 2*np.y());
    customPlot->graph(7)->setData(np_x,np_y);
    customPlot->graph(7)->setPen(QPen(Qt::yellow));
    customPlot->graph(7)->setName("Normal point (LP)");
    /// Fixe axis
    customPlot->graph(0)->rescaleAxes();
    /// Set axis equal
    customPlot->yAxis->setScaleRatio(customPlot->xAxis,1.0);
    /// Add legend
    customPlot->legend->setVisible(true);
    /// Interaction
    customPlot->setInteractions(QCP::iRangeDrag |QCP::iRangeZoom);
    /// Show
    customPlot->show();

    // Run
    int out = app.exec();

    // Clean
    delete customPlot;
#endif

    // exit
    return 0;

}
