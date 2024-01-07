#include <glog/logging.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <boost/filesystem/operations.hpp>

int main() {
    std::string strSettingsFile = boost::filesystem::current_path().string() + "/../GoPro.yaml";
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    LOG(INFO) << "strSettingsFile";
    if (!fsSettings.isOpened()) {
        std::cerr << "Failed to open settings file at: " << strSettingsFile << std::endl;
        exit(-1);
    }

    LOG(INFO) << "strSettingsFile";

    // Read and print the settings file
    float DepthMapFactor = fsSettings["DepthMapFactor"];
    float color_intrinsics_width = fsSettings["Camera.width"];
    float color_intrinsics_height = fsSettings["Camera.height"];
    float color_fx = fsSettings["Camera.fx"];
    float color_fy = fsSettings["Camera.fy"];
    float color_cx = fsSettings["Camera.cx"];
    float color_cy = fsSettings["Camera.cy"];
    LOG(INFO) << "DepthMapFactor: " << DepthMapFactor;
    LOG(INFO) << "color_intrinsics_width: " << color_intrinsics_width;
    LOG(INFO) << "color_intrinsics_height: " << color_intrinsics_height;
    LOG(INFO) << "color_fx: " << color_fx;
    LOG(INFO) << "color_fy: " << color_fy;
    LOG(INFO) << "color_cx: " << color_cx;
    LOG(INFO) << "color_cy: " << color_cy;

    return 0;
}