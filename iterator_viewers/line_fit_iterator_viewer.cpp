#include "line_fit_iterator_viewer.h"

LineFitIteratorViewer::LineFitIteratorViewer(ceres::Problem* problem, Line<double> line_gt, bool stop_every_iteration):
    m_problem(problem), m_stop_every_iteration(stop_every_iteration), m_line_gt(line_gt)
{
    m_viewer.reset(new pcl::visualization::PCLVisualizer);
    m_viewer->registerKeyboardCallback (&LineFitIteratorViewer::keyboardEventOccurred,*this);
    m_viewer->addCoordinateSystem(10);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    m_viewer->addPointCloud(pc,"points");
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,1,"points");
}
LineFitIteratorViewer::~LineFitIteratorViewer(){}

ceres::CallbackReturnType LineFitIteratorViewer::operator()(const ceres::IterationSummary& summary)
{

    // Get parameter varuables
    std::vector<double*> parameters;
    m_problem->GetParameterBlocks(&parameters);
    Line<double> line(parameters[0]);

    // Add line
    m_viewer->removeShape("line_gt");
    m_viewer->addLine(m_line_gt(-100).pcl(),m_line_gt(100).pcl(),"line_gt");
    m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"line_gt");
    m_viewer->removeShape("line");
    m_viewer->addLine(line(-100).pcl(),line(100).pcl(),"line");
    m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"line");

    // Display all errors
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < m_cf.size(); i++)
    {
        pc->push_back(m_cf[i]->point().pcl());
    }
    m_viewer->updatePointCloud(pc,"points");

    // Display
    m_viewer->addText("Press s to stop at every iteration", 30, 10, 15, 1.0f,1.0f,1.0f,"p_text");
    if (m_stop_every_iteration){
        m_viewer->addText("Press n for next iteration", 30, 25, 15, 1.0f,1.0f,1.0f,"s_text");
        m_iterate = true;
        while (m_iterate && !m_viewer->wasStopped()){
            m_viewer->spinOnce(100);
        }
    } else {
        m_viewer->spinOnce(100);
    }
    m_viewer->removeText3D("p_text");
    if (m_stop_every_iteration) m_viewer->removeText3D("s_text");

    // Exit
    return ceres::SOLVER_CONTINUE;
}

void LineFitIteratorViewer::addCostFunction(PointToLineFunctor* cf)
{
    m_cf.push_back(cf);
}

// Key callback event
void LineFitIteratorViewer::keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* junk)
{
    if (event.getKeySym() == "n" && event.keyDown()){
        m_iterate = false;
    }
    if (event.getKeySym() == "s" && event.keyDown()){
        m_iterate = false;
        m_stop_every_iteration = !m_stop_every_iteration;
    }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> LineFitIteratorViewer::viewer()
{
    return m_viewer;
}
