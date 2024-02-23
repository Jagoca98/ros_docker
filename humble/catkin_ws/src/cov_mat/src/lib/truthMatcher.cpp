#include <cov_mat/lib/truthMatcher.hpp>

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
TruthMatcher::TruthMatcher(const DetectionVectorPtr &predData,
                           const DetectionVectorPtr &gtData)
    : predData_(predData->begin(), predData->end()),
      gtData_(gtData->begin(), gtData->end()),
      n_predData_(predData->size()),
      n_gtData_(gtData->size())
{
    predMatchedData_ = std::make_shared<DetectionVector>();
    gtMatchedData_ = std::make_shared<DetectionVector>();

    predMatchedData_->clear();
    gtMatchedData_->clear();
}

TruthMatcher::~TruthMatcher()
{
}

std::vector<size_t> TruthMatcher::buildDetectionMatrix(std::vector<std::vector<double>> &detectionMatrix,
                                                       std::vector<bool> &validDetection)
{
    // Initialize costMatrix and assignment vector
    costMatrix_.resize(n_gtData_, std::vector<double>(n_predData_));

    for (size_t i = 0; i < n_gtData_; i++)
    {
        for (size_t j = 0; j < n_predData_; j++)
        {
            // Calculate the cost (distance) of assigning detection from vehicle i to vehicle j

            /// TODO Not implemented
            /** Calculate the distance between the two SE3 poses
             * Options:
             *  1. Translation error only (m)
             *  2. Weighted translation (m) + rotation (rad) error
             *  3. Unitless Frobenius norm of SE3 transformation matrix
             */
            Eigen::VectorXd predVector(3); // Assuming 3D coordinates
            Eigen::VectorXd gtVector(3);
            Eigen::VectorXd detectionErrorVector(3);

            for (size_t coord = 0; coord < 3; coord++)
            {
                predVector(coord) = predData_[j].location[coord];
                gtVector(coord) = gtData_[i].location[coord];
            }

            // Calculate detection error vector
            detectionErrorVector = predVector - gtVector;

            // Option 1 - Translation error only (m)
            costMatrix_.at(i).at(j) = detectionErrorVector.squaredNorm();

            // TODO: Option 2 - Weighted translation (m) + rotation (rad) error
            // double trans_error = detectionError.at(0) * detectionError.at(0) +
            //                      detectionError.at(1) * detectionError.at(1) +
            //                      detectionError.at(2) * detectionError.at(2);
            // const double trans_weight = 1.0;
            // const double angle_weight = 2.0;
            // costMatrix_.at(j).at(i) = trans_weight * trans_error + angle_weight * detection_error.angle();

            // TODO: Option 3 - Unitless Frobenius norm of SE3 transformation matrix
            // costMatrix_.at(j).at(i) = detection_error.se2().norm();
        }
    }

    // // Call Hungarian method to compute assignment
    loco::HungarianAssignment hungarian(costMatrix_);
    std::vector<size_t> assignment;
    hungarian.assign(assignment);

    detectionMatrix.resize(n_gtData_);       // Resize the detection matrix to the number of ground truth detections
    validDetection.resize(n_gtData_, false); // Initialize the valid detection vector to false

    // Add each assigned detection to the final detection matrix and set valid detections
    for (size_t k = 0; k < assignment.size(); k++)
    {
        // Detection from vehicle k is assigned to vehicle assigned[k]
        if (validDetection.at(k) == true)
        {
            if (verbose_dev_)
                std::cerr << "Detection already assigned" << std::endl;
            continue;
        }
        detectionMatrix.at(k) = predData_.at(assignment.at(k)).getLocation();
        validDetection.at(k) = true;
    }

    return assignment;
}

Eigen::Vector3d TruthMatcher::computeLocXYZError(const Detection &predDetection, const Detection &gtDetection)
{
    Eigen::Vector3d translationalError;

    for (size_t coord = 0; coord < 3; coord++)
    {
        translationalError(coord) = predDetection.location[coord] - gtDetection.location[coord];
    }

    return translationalError;
}

bool TruthMatcher::computeClassNameError(const Detection &predDetection, const Detection &gtDetection)
{
    if (predDetection.class_name != gtDetection.class_name)
    {
        if (verbose_dev_)
            std::cerr << "Class name mismatch: " << predDetection.class_name << " != " << gtDetection.class_name << std::endl;
        return false;
    }
    return true;
}

double TruthMatcher::computeRotationError(const Detection &predDetection, const Detection &gtDetection)
{

    // Calculate the error in the rotation between two matched detections along the World Z-axis
    double predRot = fmod(predDetection.rotation_y, 2 * M_PI);
    double gtRot = fmod(gtDetection.rotation_y, 2 * M_PI);

    predRot = (predRot < 0) ? predRot + 2 * M_PI : predRot;
    gtRot = (gtRot < 0) ? gtRot + 2 * M_PI : gtRot;

    // Calculate the module of error in the rotation as the minimun absolute difference of these three cases
    double rotationError = std::min(std::min(
                                        std::abs(predRot - gtRot),
                                        std::abs((predRot - 2 * M_PI) - gtRot)),
                                    std::abs(predRot - (gtRot - 2 * M_PI)));

    // Set the sign of the error to match the sign of the difference between the two rotations
    rotationError = (predRot < gtRot) ? rotationError : -rotationError;

    return rotationError;
}

void TruthMatcher::computeError(const Detection &predDetection, const Detection &gtDetection)
{
    locXYZError_.push_back(computeLocXYZError(predDetection, gtDetection));
    classNameError_.push_back(computeClassNameError(predDetection, gtDetection));
    rotationError_.push_back(computeRotationError(predDetection, gtDetection));
}

bool TruthMatcher::main(DetectionVectorPtr &predMatchedData,
                        DetectionVectorPtr &gtMatchedData)
{
    // Check if the number of predictions or ground truth is zero
    if (n_predData_ == 0 || n_gtData_ == 0)
    {
        if (verbose_dev_)
            std::cerr << "No predictions or ground truth data" << std::endl;
        return false;
    }

    std::vector<std::vector<double>> detectionMatrix;
    std::vector<bool> validDetection;

    std::vector<size_t> assignment;
    assignment = buildDetectionMatrix(detectionMatrix, validDetection);

    for (size_t i = 0; i < assignment.size(); i++)
    {
        auto gt = gtData_.at(i);
        auto pred = predData_.at(assignment.at(i));

        computeError(pred, gt);

        if (std::abs(locXYZError_.at(i)[0]) < locXYZ_x_thold_ &&
            std::abs(locXYZError_.at(i)[1]) < locXYZ_y_thold_ &&
            std::abs(locXYZError_.at(i)[2]) < locXYZ_z_thold_)
        {
            predMatchedData_->push_back(pred);
            gtMatchedData_->push_back(gt);
        }
    }

    predMatchedData = predMatchedData_;
    gtMatchedData = gtMatchedData_;

    if (predMatchedData->size() != gtMatchedData->size())
    {
        if (verbose_dev_)
            std::cerr << "Mismatched pred and gt data" << std::endl;
        return false;
    }

    if (predMatchedData->size() == 0 || gtMatchedData->size() == 0)
    {
        if (verbose_dev_)
            std::cerr << "No matched pred and gt data" << std::endl;
        return false;
    }

    return true;
}
