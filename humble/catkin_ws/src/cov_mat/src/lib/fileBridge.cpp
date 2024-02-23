#include <cov_mat/lib/fileBridge.hpp>

JsonBridge::JsonBridge(const std::string &filename)
{
    filename_ = filename;
    file_ = std::ifstream(filename_);

    if (!file_.is_open())
    {
        std::cerr << "Error: file " << filename_ << " not found" << std::endl;
        exit(1);
    }

    file_ >> data_;
}

JsonBridge::JsonBridge(const std::string &filename, const std::string &flushFilename)
{
    filename_ = filename;
    file_ = filename.empty() ? std::ifstream() : std::ifstream(filename);
    flushFilename_ = flushFilename;
    flushFile_ = flushFilename.empty() ? std::ofstream() : std::ofstream(flushFilename);

    if (!file_.is_open() || filename.empty())
    {
        if (verbose_dev_)
            std::cerr << "Error: file " << filename_ << " not found" << std::endl;
    }
    else
    {
        file_ >> data_;
    }
}

JsonBridge::~JsonBridge()
{
    file_.close();
}

Float3DVectorMatrixPtr JsonBridge::toAnovaWithRep()
{
    auto anova_data = std::make_shared<Float3DVectorMatrix>();

    n_blocks_ = data_.begin().value().size();
    n_groups_ = data_.size();

    for (auto group = data_.begin(); group != data_.end(); ++group)
    {
        std::vector<std::vector<float>> group_data;
        for (auto block = group.value().begin(); block != group.value().end(); ++block)
        {
            std::vector<float> block_data;
            for (auto cell = block.value().begin(); cell != block.value().end(); ++cell)
            {
                block_data.push_back(cell.value());
            }
            group_data.push_back(block_data);
        }
        anova_data->push_back(group_data);
    }

    return anova_data;
}

void JsonBridge::errorsToJson(const Float3DVectorMatrixPtr &errors)
{
    if (!flushFile_.is_open() && !flushFilename_.empty())
    {
        std::cerr << "Error: file " << flushFilename_ << " not found" << std::endl;
        exit(1);
    }

    nlohmann::json errors_json;

    for (size_t i = 0; i < errors->size(); i++)
    {
        nlohmann::json group;
        for (size_t j = 0; j < errors->at(i).size(); j++)
        {
            nlohmann::json block;
            for (size_t k = 0; k < errors->at(i).at(j).size(); k++)
            {
                block.push_back(errors->at(i).at(j).at(k));
            }
            group["block_" + std::to_string(j)] = block;

        }
        errors_json["group_" + std::to_string(i)] = group;
    }

    flushFile_ << errors_json.dump(4);
    flushFile_.close();
}

TxtBridge::TxtBridge(const std::string &filename)
{
    filename_ = filename;
    file_ = std::ifstream(filename_);
    dirPath_ = filename_;

    if (!file_.is_open())
    {
        std::cerr << "Error: file " << filename_ << " not found" << std::endl;
        exit(1);
    }
}

TxtBridge::~TxtBridge()
{
    file_.close();
}

DetectionVectorPtr TxtBridge::toTruthMatcher(const bool is_gt)
{
    auto detections = std::make_shared<DetectionVector>();

    while (getline(file_, line_, '\n'))
    {
        auto detection = toDetection(is_gt);

        // Skip DontCare detections
        if (detection.class_name == "DontCare")
            continue;

        detections->push_back(detection);
    }
    return detections;
}

Detection TxtBridge::toDetection(const bool is_gt)
{
    std::istringstream iss(line_);

    std::string className;
    std::string truncatedStr;
    std::string occludedStr;
    std::string alphaStr;
    std::vector<std::string> bbox2DStr(4);
    std::vector<std::string> dimensionsStr(3);
    std::vector<std::string> locationStr(3);
    std::string rotationYStr;
    std::string scoreStr;

    // Read values as strings
    iss >> className >> truncatedStr >> occludedStr >> alphaStr;
    for (auto &value : bbox2DStr)
        iss >> value;
    for (auto &value : dimensionsStr)
        iss >> value;
    for (auto &value : locationStr)
        iss >> value;
    iss >> rotationYStr;
    if (!is_gt)
        iss >> scoreStr;

    // Convert strings to the desired types
    bool truncated = (truncatedStr == "1");
    bool occluded = (occludedStr == "1");
    double alpha = std::stod(alphaStr);
    std::vector<double> bbox2D(bbox2DStr.size());
    std::transform(bbox2DStr.begin(), bbox2DStr.end(), bbox2D.begin(), [](const std::string &s)
                   { return std::stod(s); });
    std::vector<double> dimensions(dimensionsStr.size());
    std::transform(dimensionsStr.begin(), dimensionsStr.end(), dimensions.begin(), [](const std::string &s)
                   { return std::stod(s); });
    std::vector<double> location(locationStr.size());
    std::transform(locationStr.begin(), locationStr.end(), location.begin(), [](const std::string &s)
                   { return std::stod(s); });
    double rotationY = std::stod(rotationYStr);
    double score;
    if (!is_gt)
        score = std::stod(scoreStr);

    // Create Detection object using the converted values
    if (is_gt)
        return Detection(className, truncated, occluded, alpha,
                         bbox2D, dimensions, location, rotationY);
    else
        return Detection(className, truncated, occluded, alpha,
                         bbox2D, dimensions, location, rotationY, score);
}