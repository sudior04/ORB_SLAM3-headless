#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>

#include <opencv2/opencv.hpp>
#include "System.h"
#include "BackendSender.h"

using namespace std;

void LoadImages(
    const string &image_list_file,
    const string &timestamp_file,
    vector<string> &image_names,
    vector<double> &timestamps
)
{
    ifstream f_images(image_list_file);
    ifstream f_times(timestamp_file);

    string line;

    while (getline(f_images, line))
    {
        if (!line.empty())
            image_names.push_back(line);
    }

    while (getline(f_times, line))
    {
        if (!line.empty())
            timestamps.push_back(stod(line));
    }
}

string GenerateDatasetSessionId(const string& datasetName)
{
    time_t now = time(nullptr);

    tm timeInfo{};
    localtime_r(&now, &timeInfo);

    char buffer[64];

    strftime(
        buffer,
        sizeof(buffer),
        "%Y%m%d_%H%M%S",
        &timeInfo
    );

    return "dataset_" + datasetName + "_" + string(buffer);
}

int main(int argc, char **argv)
{
    if (argc != 6)
    {
        cerr << endl;
        cerr << "Usage: ./mono_image_headless path_to_vocabulary path_to_settings path_to_image_folder image_list.txt timestamps.txt" << endl;
        cerr << endl;
        return 1;
    }

    string voc_file = argv[1];
    string settings_file = argv[2];
    string image_folder = argv[3];
    string image_list_file = argv[4];
    string timestamp_file = argv[5];

    vector<string> image_names;
    vector<double> timestamps;

    LoadImages(image_list_file, timestamp_file, image_names, timestamps);

    if (image_names.empty())
    {
        cerr << "ERROR: image list is empty." << endl;
        return 1;
    }

    if (timestamps.empty())
    {
        cerr << "ERROR: timestamps file is empty." << endl;
        return 1;
    }

    if (image_names.size() != timestamps.size())
    {
        cerr << "ERROR: image count != timestamp count" << endl;
        cerr << "Images: " << image_names.size() << endl;
        cerr << "Timestamps: " << timestamps.size() << endl;
        return 1;
    }

    ORB_SLAM3::System SLAM(
        voc_file,
        settings_file,
        ORB_SLAM3::System::MONOCULAR,
        false
    );

    string sessionId = GenerateDatasetSessionId("image_dataset");

    ORB_SLAM3::BackendSender backendSender(
        "192.168.1.4:5000",
        sessionId,
        "device_rov"
    );

    for (size_t i = 0; i < image_names.size(); i++)
    {
        string image_path = image_folder + "/" + image_names[i];

        cv::Mat im = cv::imread(image_path, cv::IMREAD_UNCHANGED);

        if (im.empty())
        {
            cerr << "ERROR: failed to load image: " << image_path << endl;
            continue;
        }

        SLAM.TrackMonocular(im, timestamps[i]);

        if (i % 30 == 0)
        {
            cout << "Processed frame " << i << " / " << image_names.size() << endl;
        }
    }

    SLAM.Shutdown();

    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    backendSender.UploadKeyFrameTrajectory(
    "KeyFrameTrajectory.txt",
    "image_dataset",
    "monocular"
    );

    cout << "Done." << endl;
    cout << "Saved: CameraTrajectory.txt" << endl;
    cout << "Saved: KeyFrameTrajectory.txt" << endl;

    return 0;
}