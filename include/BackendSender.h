#ifndef BACKEND_SENDER_H
#define BACKEND_SENDER_H

#include <string>
#include <sophus/se3.hpp>

namespace ORB_SLAM3
{

class BackendSender
{
public:
    BackendSender(
        const std::string& backendUrl,
        const std::string& sessionId,
        const std::string& deviceId = ""
    );

    void SendRealtimePose(
        double timestamp,
        const Sophus::SE3f& Tcw,
        const std::string& cameraSource,
        float fps,
        int width,
        int height,
        const std::string& sensorMode
    );

    void UploadKeyFrameTrajectory(
        const std::string& filePath,
        const std::string& datasetName,
        const std::string& sensorMode
    );

private:
    std::string mBackendUrl;
    std::string mSessionId;
    std::string mDeviceId;

    void PostJsonAsync(const std::string& endpoint, const std::string& json);
};

}

#endif