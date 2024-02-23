#ifndef DETECTION_DRAWER_HPP
#define DETECTION_DRAWER_HPP

#include <Eigen/Geometry>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <cov_mat/lib/Detection.hpp>

// Alias for vector of Detections
using DetectionVector = std::vector<Detection>;
using DetectionVectorPtr = std::shared_ptr<std::vector<Detection>>;

class DetectionDrawer
{
public:
    static pcl::visualization::PCLVisualizer::Ptr viewer;

    static void setupViewer();

    static void removeAllDetections();

    static void addDetection(const Detection &detection,
                             const bool &is_gt, const int &id);

    static void addDetections(const DetectionVectorPtr &detections,
                              const bool &is_gt);

    static void set_verbose(bool verbose) { verbose_ = verbose; };
    static bool get_verbose() { return verbose_; };
    static void set_verbose_dev(bool verbose_dev) { verbose_dev_ = verbose_dev; };
    static bool get_verbose_dev() { return verbose_dev_; };

private:
    static bool verbose_;
    static bool verbose_dev_;

    static void drawFloorPlane();
};

#endif // DETECTION_DRAWER_HPP