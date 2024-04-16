#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void extractImagesFromBag(const std::string& bag_file, const std::string& output_dir, const std::string& image_topic)
{
    rosbag::Bag bag;
    try
    {
        bag.open(bag_file, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery(image_topic));

        cv_bridge::CvImagePtr cv_ptr;
        int frame_number = 0;

        for (const rosbag::MessageInstance& msg : view)
        {
            sensor_msgs::Image::ConstPtr image_msg = msg.instantiate<sensor_msgs::Image>();
            if (image_msg)
            {
                try
                {
                    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
                    std::string image_filename = output_dir + "/frame" + std::to_string(frame_number) + ".jpg";
                    cv::imwrite(image_filename, cv_ptr->image);
                    frame_number++;
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("Error converting image: %s", e.what());
                }
            }
        }
    }
    catch (rosbag::BagException& e)
    {
        ROS_ERROR("Error opening bag file: %s", e.what());
    }
    bag.close();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag_image_extractor");
    if (argc != 4)
    {
        ROS_ERROR("Usage: %s <bag_file> <output_directory> <image_topic>", argv[0]);
        return 1;
    }

    std::string bag_file = argv[1];
    std::string output_dir = argv[2];
    std::string image_topic = argv[3];

    extractImagesFromBag(bag_file, output_dir, image_topic);

    ROS_INFO("Images extracted successfully!");
    return 0;
}
