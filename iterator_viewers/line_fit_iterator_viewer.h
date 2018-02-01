#ifndef LINE_FIT_ITERATOR_VIEWER_H
#define LINE_FIT_ITERATOR_VIEWER_H

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

// Ceres
#include <ceres/ceres.h>

// Local
#include "tools/line.h"
#include "functors/point_to_line_functor.h"

class LineFitIteratorViewer: public ceres::IterationCallback {
public:
    LineFitIteratorViewer(ceres::Problem* problem, Line<double> line_gt, bool stop_every_iteration = false);
    ~LineFitIteratorViewer();

    ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary);

    void addCostFunction(PointToLineFunctor* cf);

    // Key callback event
    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* junk);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer();

private:
    ceres::Problem* m_problem;
    std::vector<PointToLineFunctor*> m_cf;
    bool m_stop_every_iteration;
    bool m_iterate;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
    Line<double> m_line_gt;

};

#endif // LINE_FIT_ITERATOR_VIEWER_H
