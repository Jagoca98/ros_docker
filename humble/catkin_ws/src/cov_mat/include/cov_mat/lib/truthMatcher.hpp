#ifndef TRUTHMATCHER_H
#define TRUTHMATCHER_H

#include <boost/filesystem.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cov_mat/lib/hungarian.hpp>
#include <cov_mat/lib/Detection.hpp>

// Alias for vector of Detections
using DetectionVector = std::vector<Detection>;
using DetectionVectorPtr = std::shared_ptr<DetectionVector>;

/* Matrix alias for 2D vectors */
template <typename T>
using Matrix = std::vector<std::vector<T>>;

class TruthMatcher
{
public:
    /**
     * @brief Constructs a TruthMatcher object.
     *
     * This constructor initializes a TruthMatcher object with the given predicted data and ground truth data.
     * It also initializes the number of predicted data and ground truth data.
     * The matched data vectors are cleared.
     *
     * @param predData A shared pointer to the predicted data vector.
     * @param gtData A shared pointer to the ground truth data vector.
     */
    TruthMatcher(const DetectionVectorPtr &predData,
                 const DetectionVectorPtr &gtData);

    /**
     * @brief Destroy the Truth Matcher object
     */
    ~TruthMatcher();

    /**
     * @brief Main function to run the truth matcher
     *
     * @param predMatchedData Pointer to the vector of matched predictions
     * @param gtMatchedData Pointer to the vector of matched ground truth
     * @return true If the truth matcher ran successfully. False otherwise
     */
    bool main(DetectionVectorPtr &predMatchedData,
              DetectionVectorPtr &gtMatchedData);

    void set_verbose(bool verbose) { verbose_ = verbose; };
    bool get_verbose() { return verbose_; };
    void set_verbose_dev(bool verbose_dev) { verbose_dev_ = verbose_dev; };
    bool get_verbose_dev() { return verbose_dev_; };

    void set_locXYZ_x_thold(float locXYZ_x_thold) { locXYZ_x_thold_ = locXYZ_x_thold; };
    float get_locXYZ_x_thold() { return locXYZ_x_thold_; };
    void set_locXYZ_y_thold(float locXYZ_y_thold) { locXYZ_y_thold_ = locXYZ_y_thold; };
    float get_locXYZ_y_thold() { return locXYZ_y_thold_; };
    void set_locXYZ_z_thold(float locXYZ_z_thold) { locXYZ_z_thold_ = locXYZ_z_thold; };
    float get_locXYZ_z_thold() { return locXYZ_z_thold_; };
    void set_rotation_thold(float rotation_thold) { rotation_thold_ = rotation_thold; };
    float get_rotation_thold() { return rotation_thold_; };

private:
    bool verbose_dev_ = false;
    bool verbose_ = true;

    DetectionVector predData_;
    DetectionVector gtData_;

    DetectionVectorPtr predMatchedData_;
    DetectionVectorPtr gtMatchedData_;

    size_t n_predData_;
    size_t n_gtData_;

    Matrix<double> costMatrix_;

    // Errors
    std::vector<Eigen::Vector3d> locXYZError_; // Δx, Δy, Δz
    std::vector<bool> classNameError_;             // True if class names match
    std::vector<double> rotationError_;            // φ error in the rotation along the World Z-axis

    float locXYZ_x_thold_ = 5;
    float locXYZ_y_thold_ = 5;
    float locXYZ_z_thold_ = 5;
    float rotation_thold_ = 0.15;

    /**
     * @brief Build the detection matrix and preform the assignment using the Hungarian algorithm
     *
     * @param detectionMatrix
     * @param validDetection
     * @return std::vector<size_t>  The assignment vector
     */
    std::vector<size_t> buildDetectionMatrix(std::vector<std::vector<double>> &detectionMatrix,
                                             std::vector<bool> &validDetection);

    /**
     * @brief Compute error location X, Y, Z between two matched detections
     *
     * @param predDetection
     * @param gtDetection
     * @return Eigen::Vector3d. The error vector (Δx, Δy, Δz)
     */
    Eigen::Vector3d computeLocXYZError(const Detection &predDetection, const Detection &gtDetection);

    /**
     * @brief Compute the error in the class name between two matched detections
     *
     * @param predDetection
     * @param gtDetection
     * @return Bool. True if class names match, false otherwise
     */
    bool computeClassNameError(const Detection &predDetection, const Detection &gtDetection);

    /**
     * @brief Compute the error in the rotation between two matched detections along the World Z-axis
     *
     * @param predDetection
     * @param gtDetection
     * @return double. The φ error in the rotation along the World Z-axis
     */
    double computeRotationError(const Detection &predDetection, const Detection &gtDetection);

    /**
     * @brief Compute the error between two matched detections
     *
     * @param predDetection
     * @param gtDetection
     */
    void computeError(const Detection &predDetection, const Detection &gtDetection);
};

#endif // TRUTHMATCHER_H