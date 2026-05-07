#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>
#include "System.h"

using namespace std;

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

    cout << "USB camera opened. Running ORB-SLAM3 headless..." << endl;

    auto t_start = chrono::steady_clock::now();
    int frame_id = 0;

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

        SLAM.TrackMonocular(frame, timestamp);

        if (frame_id % 30 == 0)
        {
            cout << "Processed frame: " << frame_id
                 << " timestamp: " << timestamp << endl;
        }

        frame_id++;

        if (frame_id >= 900)
        {
            cout << "Reached 900 frames. Stop." << endl;
            break;
        }
    }

    SLAM.Shutdown();

    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_usb.txt");

    cout << "Done." << endl;
    cout << "Saved: KeyFrameTrajectory_usb.txt" << endl;

    return 0;
}