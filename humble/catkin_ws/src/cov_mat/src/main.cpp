#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <boost/filesystem.hpp>

#include <cov_mat/lib/anova.hpp>
#include <cov_mat/lib/fileBridge.hpp>
#include <cov_mat/lib/truthMatcher.hpp>
#include <cov_mat/lib/detectionDrawer.hpp>

int main()
{
    bool verbose_dev = true; // Set to true to print debug messages
    bool render = false;     // Set to true to render the detections

    boost::filesystem::path directoryPath = boost::filesystem::current_path() / boost::filesystem::path("tmp_dev/pred_kitti_second");
    std::vector<boost::filesystem::directory_entry> files;

    // Populate the files vector with regular files in the directory
    for (const auto &entry : boost::filesystem::directory_iterator(directoryPath))
    {
        if (boost::filesystem::is_regular_file(entry.path()))
        {
            files.push_back(entry);
        }
    }

    // Sort files based on their names
    std::sort(files.begin(), files.end(), [](const auto &entry1, const auto &entry2)
              { return entry1.path().filename() < entry2.path().filename(); });

    for (const auto &entry : files)
    {
        if (boost::filesystem::is_regular_file(entry.path()))
        {
            // Assuming file names follow the pattern "000001.txt"
            std::string fileName = entry.path().filename().string();

            // if(fileName != "000010.txt")
            //     continue;

            // Construct paths for pred and gt files
            boost::filesystem::path predPath = boost::filesystem::current_path() / boost::filesystem::path("tmp_dev/pred_kitti_second/") / fileName;
            boost::filesystem::path gtPath = boost::filesystem::current_path() / boost::filesystem::path("tmp_dev/gt_kitti_second/") / fileName;

            if (verbose_dev)
                std::cout << "\nProcessing " << fileName << std::endl;

            // Create TxtBridge instances for pred and gt
            TxtBridge pred(predPath.string());
            TxtBridge gt(gtPath.string());

            // Convert TxtBridge instances to TruthMatcher data
            DetectionVectorPtr predData = pred.toTruthMatcher();
            DetectionVectorPtr gtData = gt.toTruthMatcher(true);

            // Create TruthMatcher instance to match each ground truth instance with each prediction
            TruthMatcher truthMatcher(predData, gtData);

            std::cout << "Pred data size before: " << predData->size() << std::endl;
            std::cout << "GT data size before: " << gtData->size() << std::endl;

            truthMatcher.main(predData, gtData);

            std::cout << "Pred data size after: " << predData->size() << std::endl;
            std::cout << "GT data size after: " << gtData->size() << std::endl;

            if (render)
            {
                // Set up the viewer
                DetectionDrawer::setupViewer();

                // Add detections to viewer
                DetectionDrawer::addDetections(gtData, true);
                DetectionDrawer::addDetections(predData, false);

                // Set viewer options
                while (!DetectionDrawer::viewer->wasStopped())
                {
                    DetectionDrawer::viewer->spinOnce();
                }
            }
        }
    }

    auto anovaData = JsonBridge("/home/jaime/Desktop/tfm/tfm_ros2_ws/src/cov_mat/src/tmp_dev/CropFertilizer.json").toAnovaWithRep();
    std::cout << "ANOVA data size: " << anovaData->size() << std::endl;

    // Flush into a JSON file
    JsonBridge("", "/home/jaime/Desktop/tfm/tfm_ros2_ws/src/cov_mat/src/tmp_dev/CropFertilizer_error.json").errorsToJson(anovaData);

    std::cout << "Done" << std::endl;

    return 0;
}