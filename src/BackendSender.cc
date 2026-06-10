#include "BackendSender.h"

#include <curl/curl.h>
#include <sstream>
#include <thread>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ORB_SLAM3
{

BackendSender::BackendSender(
    const std::string& backendUrl,
    const std::string& sessionId,
    const std::string& deviceId
)
{
    mBackendUrl = backendUrl;
    mSessionId = sessionId;
    mDeviceId = deviceId;
}

void BackendSender::PostJsonAsync(
    const std::string& endpoint,
    const std::string& json
)
{
    std::string url = mBackendUrl + endpoint;

    std::thread([url, json]() {
        CURL* curl = curl_easy_init();

        if (!curl)
            return;

        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 100L);

        CURLcode res = curl_easy_perform(curl);

        if (res != CURLE_OK)
        {
            std::cerr << "[BackendSender] POST failed: "
                      << curl_easy_strerror(res)
                      << std::endl;
        }

        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
    }).detach();
}

void BackendSender::SendRealtimePose(
    double timestamp,
    const Sophus::SE3f& Tcw,
    const std::string& cameraSource,
    float fps,
    int width,
    int height,
    const std::string& sensorMode
)
{
    if (mBackendUrl.empty() || mSessionId.empty())
        return;

    Sophus::SE3f Twc = Tcw.inverse();

    Eigen::Vector3f t = Twc.translation();
    Eigen::Quaternionf q(Twc.rotationMatrix());

    std::stringstream ss;

    ss << "{";
    ss << "\"sessionId\":\"" << mSessionId << "\",";
    ss << "\"mode\":\"realtime\",";
    ss << "\"deviceId\":\"" << mDeviceId << "\",";
    ss << "\"status\":\"running\",";

    ss << "\"cameraSource\":\"" << cameraSource << "\",";
    ss << "\"fps\":" << fps << ",";
    ss << "\"width\":" << width << ",";
    ss << "\"height\":" << height << ",";
    ss << "\"sensorMode\":\"" << sensorMode << "\",";

    ss << "\"frame\":{";
    ss << "\"timestamp\":" << timestamp << ",";
    ss << "\"pose\":{";
    ss << "\"tx\":" << t.x() << ",";
    ss << "\"ty\":" << t.y() << ",";
    ss << "\"tz\":" << t.z() << ",";
    ss << "\"qx\":" << q.x() << ",";
    ss << "\"qy\":" << q.y() << ",";
    ss << "\"qz\":" << q.z() << ",";
    ss << "\"qw\":" << q.w();
    ss << "},";
    ss << "\"trackingState\":\"OK\"";
    ss << "}";

    ss << "}";

    PostJsonAsync("/api/slam/realtime", ss.str());
}

void BackendSender::UploadKeyFrameTrajectory(
    const std::string& filePath,
    const std::string& datasetName,
    const std::string& sensorMode
)
{
    if (mBackendUrl.empty() || mSessionId.empty())
        return;

    std::string url = mBackendUrl + "/api/slam/dataset/upload-keytrach";

    CURL* curl = curl_easy_init();

    if (!curl)
        return;

    curl_mime* mime = curl_mime_init(curl);

    curl_mimepart* part;

    part = curl_mime_addpart(mime);
    curl_mime_name(part, "sessionId");
    curl_mime_data(part, mSessionId.c_str(), CURL_ZERO_TERMINATED);

    part = curl_mime_addpart(mime);
    curl_mime_name(part, "mode");
    curl_mime_data(part, "dataset", CURL_ZERO_TERMINATED);

    part = curl_mime_addpart(mime);
    curl_mime_name(part, "datasetName");
    curl_mime_data(part, datasetName.c_str(), CURL_ZERO_TERMINATED);

    part = curl_mime_addpart(mime);
    curl_mime_name(part, "sensorMode");
    curl_mime_data(part, sensorMode.c_str(), CURL_ZERO_TERMINATED);

    part = curl_mime_addpart(mime);
    curl_mime_name(part, "file");
    curl_mime_filedata(part, filePath.c_str());

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 5000L);

    CURLcode res = curl_easy_perform(curl);

    if (res != CURLE_OK)
    {
        std::cerr << "[BackendSender] upload failed: "
                  << curl_easy_strerror(res)
                  << std::endl;
    }
    else
    {
        std::cout << "[BackendSender] uploaded keyframe trajectory: "
                  << filePath << std::endl;
    }

    curl_mime_free(mime);
    curl_easy_cleanup(curl);
}

}