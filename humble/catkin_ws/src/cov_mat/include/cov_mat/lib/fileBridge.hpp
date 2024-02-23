#ifndef FILEBRIDGE_H
#define FILEBRIDGE_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>

#include <boost/filesystem.hpp>

#include <cov_mat/lib/json.hpp>
#include <cov_mat/lib/Detection.hpp>

// Alias for 3D vector of floats
using Float3DVectorMatrix = std::vector<std::vector<std::vector<float>>>;
using Float3DVectorMatrixPtr = std::shared_ptr<std::vector<std::vector<std::vector<float>>>>;

// Alias for vector of Detections
using DetectionVector = std::vector<Detection>;
using DetectionVectorPtr = std::shared_ptr<std::vector<Detection>>;

// Abstract class for file handlers
class FileBridgeBase
{
public:
    void setFilename(const std::string &filename)

    {
        filename_ = filename;
        file_ = std::ifstream(filename);
    }

    void set_file(const std::string &filename) { file_ = std::ifstream(filename); };           // Setter for file_
    std::string get_filename() { return filename_; };                                          // Getter for filename_
    void set_flushFile(const std::string &filename) { flushFile_ = std::ofstream(filename); }; // Setter for flushFile_
    std::string get_flushFilename() { return flushFilename_; };                                // Getter for flushFile_

    void set_verbose(const bool verbose) { verbose_ = verbose; };                // Setter for verbose_
    bool get_verbose() { return verbose_; };                                     // Getter for verbose_
    void set_verbose_dev(const bool verbose_dev) { verbose_dev_ = verbose_dev; }; // Setter for verbose_dev_
    bool get_verbose_dev() { return verbose_dev_; };                              // Getter for verbose_dev_

protected:
    bool verbose_ = false;
    bool verbose_dev_ = false;

    std::string filename_;
    std::ifstream file_;
    std::string flushFilename_;
    std::ofstream flushFile_;

    std::string dirPath_;
};

class JsonBridge : public FileBridgeBase
{
public:
    /**
     * @brief Construct a new JsonBridge object
     */
    JsonBridge(const std::string &filename);

    JsonBridge(const std::string &filename, const std::string &flushFilename);

    /**
     * @brief Destroy the JsonBridge object
     */
    ~JsonBridge();

    /**
     * @brief Convert the JSON data to datatype suitable for ANOVA with replication library
     */
    Float3DVectorMatrixPtr toAnovaWithRep();

    /**
     * @brief Convert the list of errors in measurements into a JSON file
     */
    void errorsToJson(const Float3DVectorMatrixPtr &errors);

    auto get_data() { return data_; }; // Getter for data_

private:
    nlohmann::json data_;

    size_t n_groups_;
    size_t n_blocks_;
};

class TxtBridge : public FileBridgeBase
{
public:
    /**
     * @brief Construct a new TxtBridge object
     */
    TxtBridge(const std::string &filename);

    /**
     * @brief Destroy the TxtBridge object
     */
    ~TxtBridge();

    DetectionVectorPtr toTruthMatcher(const bool is_gt = false);

private:
    Detection toDetection(const bool is_gt = false);

    std::string data_;
    std::string line_;
};

#endif // FILEBRIDGE_H