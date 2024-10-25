#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

int main(int argc, char** argv) {
    // Check if the correct number of arguments is provided
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <input_bag> <image_topic> <output_folder>" << std::endl;
        return -1;
    }

    std::string bag_file = argv[1];
    std::string image_topic = argv[2];
    std::string output_folder = argv[3];

    // Create the output directory if it doesn't exist
    boost::filesystem::path dir(output_folder);
    if (!boost::filesystem::exists(dir)) {
        boost::filesystem::create_directories(dir);
    }

    rosbag::Bag bag;
    try {
        bag.open(bag_file, rosbag::bagmode::Read);
    } catch (rosbag::BagException& e) {
        std::cerr << "Error opening bag file: " << e.what() << std::endl;
        return -1;
    }

    rosbag::View view(bag, rosbag::TopicQuery(image_topic));
    cv_bridge::CvImagePtr cv_ptr;
    int image_count = 0;

    // Iterate through each message in the topic
    for (const rosbag::MessageInstance& msg : view) {
        sensor_msgs::ImageConstPtr img_msg = msg.instantiate<sensor_msgs::Image>();
        if (img_msg != nullptr) {
            try {
                // Convert the ROS image to OpenCV format
                cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

                // Construct the filename and save the image
                std::stringstream ss;
                ss << output_folder << "/image_" << std::setw(5) << std::setfill('0') << image_count << ".png";
                cv::imwrite(ss.str(), cv_ptr->image);

                std::cout << "Saved " << ss.str() << std::endl;
                image_count++;
            } catch (cv_bridge::Exception& e) {
                std::cerr << "cv_bridge exception: " << e.what() << std::endl;
            }
        }
    }

    bag.close();
    std::cout << "Finished extracting images. Total images: " << image_count << std::endl;

    return 0;
}