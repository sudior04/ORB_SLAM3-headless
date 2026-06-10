#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>
#include "System.h"
#include <sophus/se3.hpp>
#include "BackendSender.h"

using namespace std;

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

    return datasetName + "_" + string(buffer);
}

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        cerr << "Usage: ./mono_usb_headless path_to_vocabulary path_to_settings camera_id" << endl;
        return 1;
    }

    string voc_file = argv[1];
    string settings_file = argv[2];
    int camera_id = atoi(argv[3]);

    cv::VideoCapture cap(camera_id);

    if (!cap.isOpened())
    {
        cerr << "ERROR: Cannot open camera id " << camera_id << endl;
        return 1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    ORB_SLAM3::System SLAM(
        voc_file,
        settings_file,
        ORB_SLAM3::System::MONOCULAR,
        false
    );

    string sessionId = GenerateDatasetSessionId("camera");

    ORB_SLAM3::BackendSender backendSender(
        "http://localhost:5000",
        sessionId,
        "device_rov"
    );

    cout << "USB camera opened. Running ORB-SLAM3 headless..." << endl;

    auto t_start = chrono::steady_clock::now();
    int frame_id = 0;

    double last_log_time = -1.0;

    int frameCounter = 0;

    while (true)
    {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty())
        {
            cerr << "ERROR: empty frame" << endl;
            break;
        }

        auto now = chrono::steady_clock::now();
        double timestamp = chrono::duration<double>(now - t_start).count();

        Sophus::SE3f Tcw = SLAM.TrackMonocular(frame, timestamp);

        int trackingState = SLAM.GetTrackingState();

       if (trackingState == ORB_SLAM3::Tracking::OK)
        {
            frameCounter++;

            if (frameCounter % 2 == 0)
            {
                backendSender.SendRealtimePose(
                    timestamp,
                    Tcw,
                    "camera_2",
                    30.0,
                    frame.cols,
                    frame.rows,
                    "monocular"
                );
            }
        }

        if (timestamp - last_log_time >= 0.5)
        {
            last_log_time = timestamp;

            if (!Tcw.matrix().isZero())
            {
                Sophus::SE3f Twc = Tcw.inverse();
                Eigen::Vector3f pos = Twc.translation();

                cout << "POSE "
                    << timestamp << " "
                    << pos.x() << " "
                    << pos.y() << " "
                    << pos.z()
                    << endl;
            }
            else
            {
                cout << "LOST " << timestamp << endl;
            }
        }
    }

    SLAM.Shutdown();

    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_usb.txt");

    cout << "Done." << endl;
    cout << "Saved: KeyFrameTrajectory_usb.txt" << endl;

    return 0;
}