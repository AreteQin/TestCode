#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "video_to_bag");
    if (argc < 3) {
        ROS_ERROR("Usage: video_to_bag <input_video> <output_bag>");
        return 1;
    }
    std::string video_file = argv[1];
    std::string bag_file = argv[2];

    // Open the video file
    cv::VideoCapture cap(video_file);
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open video file: %s", video_file.c_str());
        return 1;
    }

    double fps = cap.get(cv::CAP_PROP_FPS);
    ros::Time start_time = ros::Time::now();
    rosbag::Bag bag;
    bag.open(bag_file, rosbag::BagMode::Write);

    cv::Mat frame;
    int frame_count = 0;
    ros::Duration frame_duration(1.0 / fps);

    while (ros::ok() && cap.read(frame)) {
        ros::Time timestamp = start_time + frame_duration * frame_count;
        std_msgs::Header header;
        header.stamp = timestamp;
        header.frame_id = "camera";

        // Convert OpenCV image to ROS image message
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

        // Write the image message into the bag under the topic /camera/image_raw
        bag.write("/camera/image_raw", timestamp, img_msg);

        frame_count++;
    }

    bag.close();
    ROS_INFO("Finished writing bag file: %s", bag_file.c_str());
    return 0;
}
