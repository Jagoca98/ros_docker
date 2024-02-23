#ifndef DETECTION_HPP
#define DETECTION_HPP

#include <string>
#include <vector>
#include <iostream>

class Detection
{
public:
    // Member variables
    std::string class_name;
    bool truncated;
    bool occluded;
    double alpha;
    std::vector<double> bbox2D;
    std::vector<double> dimensions;
    std::vector<double> location;
    double rotation_y;
    double score;

    // Default constructor
    Detection() : truncated(false), occluded(false), alpha(0.0), rotation_y(0.0), score(0.0) {}

    // Parameterized constructor for predictions
    Detection(std::string t, bool tr, bool oc, double a, const std::vector<double> &bbox,
              const std::vector<double> &dims, const std::vector<double> &loc, double rot_y, double s)
        : class_name(std::move(t)), truncated(tr), occluded(oc), alpha(a),
          bbox2D(bbox), dimensions(dims), location(loc), rotation_y(rot_y), score(s) {}

    // Parameterized constructor for ground truth
    Detection(std::string t, bool tr, bool oc, double a, const std::vector<double> &bbox,
              const std::vector<double> &dims, const std::vector<double> &loc, double rot_y)
        : class_name(std::move(t)), truncated(tr), occluded(oc), alpha(a),
          bbox2D(bbox), dimensions(dims), location(loc), rotation_y(rot_y), score(-1.0) {}

    // Get location
    std::vector<double> getLocation() { return location; }

    // Print function
    void print() const
    {
        std::cout << "Class name: " << class_name << std::endl;
        std::cout << "Truncated: " << truncated << std::endl;
        std::cout << "Occluded: " << occluded << std::endl;
        std::cout << "Alpha: " << alpha << std::endl;

        std::cout << "Bounding Box 2D: ";
        for (const auto &value : bbox2D)
        {
            std::cout << value << " ";
        }
        std::cout << std::endl;

        std::cout << "Dimensions: ";
        for (const auto &value : dimensions)
        {
            std::cout << value << " ";
        }
        std::cout << std::endl;

        std::cout << "Location: ";
        for (const auto &value : location)
        {
            std::cout << value << " ";
        }
        std::cout << std::endl;

        std::cout << "Rotation Y: " << rotation_y << std::endl;
        std::cout << "Score: " << score << std::endl;
    }

    friend std::ostream &operator<<(std::ostream &os, const Detection &detection)
    {
        os << "Class name: " << detection.class_name << "\n"
           << "Truncated: " << detection.truncated << "\n"
           << "Occluded: " << detection.occluded << "\n"
           << "Alpha: " << detection.alpha << "\n"
           << "Bounding Box 2D: ";

        for (const auto &value : detection.bbox2D)
        {
            os << value << " ";
        }
        os << "\nDimensions: ";

        for (const auto &value : detection.dimensions)
        {
            os << value << " ";
        }
        os << "\nLocation: ";

        for (const auto &value : detection.location)
        {
            os << value << " ";
        }

        os << "\nRotation Y: " << detection.rotation_y << "\n"
           << "Score: " << detection.score << std::endl;

        return os;
    }
};

#endif // DETECTION_HPP