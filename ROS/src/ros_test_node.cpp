#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_to_bag");
    ros::NodeHandle nh;                       // <-- starts ROS time

    if (argc < 3) {
        ROS_ERROR("Usage: video_to_bag <input_video> <output_bag>");
        return 1;
    }

    std::string video_file = argv[1];
    std::string bag_file   = argv[2];

    cv::VideoCapture cap(video_file);
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open video: %s", video_file.c_str());
        return 1;
    }

    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0) { fps = 30; ROS_WARN("FPS unknown, defaulting to 30"); }

    ros::Time     start_time = ros::Time::now();
    rosbag::Bag   bag;
    bag.open(bag_file, rosbag::BagMode::Write);

    cv::Mat frame;
    int      frame_count   = 0;
    ros::Duration frame_dt(1.0 / fps);

    while (ros::ok() && cap.read(frame)) {
        ros::Time timestamp = start_time + frame_dt * frame_count;

        std_msgs::Header header;
        header.stamp    = timestamp;
        header.frame_id = "camera";

        sensor_msgs::ImagePtr img_msg =
            cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

        bag.write("/camera/image_raw", timestamp, img_msg);
        ++frame_count;
    }

    bag.close();
    ROS_INFO("Bag written to %s", bag_file.c_str());
    return 0;
}

