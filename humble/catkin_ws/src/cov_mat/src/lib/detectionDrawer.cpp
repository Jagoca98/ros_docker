#include <cov_mat/lib/detectionDrawer.hpp>

// Define the static member outside the class
pcl::visualization::PCLVisualizer::Ptr DetectionDrawer::viewer;
bool DetectionDrawer::verbose_ = false;
bool DetectionDrawer::verbose_dev_ = false;

void DetectionDrawer::setupViewer()
{
    viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Detection Viewer"));

    viewer->setBackgroundColor(0.0, 0.0, 0.0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    drawFloorPlane();
}

void DetectionDrawer::removeAllDetections()
{
    viewer->removeAllShapes();
}

void DetectionDrawer::addDetection(const Detection &detection,
                                   const bool &is_gt, const int &id)
{
    if (is_gt)
    {
        viewer->addCube(
            detection.location[0] - detection.dimensions[0] / 2.0,
            detection.location[0] + detection.dimensions[0] / 2.0,
            detection.location[1] - detection.dimensions[1] / 2.0,
            detection.location[1] + detection.dimensions[1] / 2.0,
            detection.location[2] - detection.dimensions[2] / 2.0,
            detection.location[2] + detection.dimensions[2] / 2.0,
            0.0, 1.0, 0.0, // RGB color for the box
            detection.class_name + "_gt_" + std::to_string(id));

        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                            detection.class_name + "_gt_" + std::to_string(id));
    }
    else
    {
        viewer->addCube(
            detection.location[0] - detection.dimensions[0] / 2.0,
            detection.location[0] + detection.dimensions[0] / 2.0,
            detection.location[1] - detection.dimensions[1] / 2.0,
            detection.location[1] + detection.dimensions[1] / 2.0,
            detection.location[2] - detection.dimensions[2] / 2.0,
            detection.location[2] + detection.dimensions[2] / 2.0,
            1.0, 0.0, 0.0, // RGB color for the box
            detection.class_name + "_pred_" + std::to_string(id));

        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                            detection.class_name + "_pred_" + std::to_string(id));
    }
}

void DetectionDrawer::addDetections(const DetectionVectorPtr &detections,
                                    const bool &is_gt)
{
    if (detections->size() <= 0)
    {
        if (verbose_dev_)
            std::cerr << "DetectionDrawer::addDetections() -> No detections to add" << std::endl;
        return;
    }

    for (size_t i = 0; i < detections->size(); i++)
    {
        addDetection(detections->at(i), is_gt, i);
    }
}

void DetectionDrawer::drawFloorPlane()
{
    // Define the parameters of the floor plane (you can adjust these values)
    double a = 0.0;
    double b = -1.0;
    double c = 0.0; // normal vector to the plane
    double d = 0.0; // distance from the origin to the plane

    // Set color for the floor (you can change this to suit your visualization)
    double floor_color_r = 0.8;
    double floor_color_g = 0.8;
    double floor_color_b = 0.8;

    // Create a pcl::ModelCoefficients object and set its values
    pcl::ModelCoefficients::Ptr floor_coefficients(new pcl::ModelCoefficients);
    floor_coefficients->values.resize(4);
    floor_coefficients->values[0] = a;
    floor_coefficients->values[1] = b;
    floor_coefficients->values[2] = c;
    floor_coefficients->values[3] = d;

    // Add the floor plane to the visualizer
    viewer->addPlane(*floor_coefficients, "floor");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, floor_color_r, floor_color_g, floor_color_b, "floor");
}
