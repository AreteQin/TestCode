#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.hpp"

class CameraStreamTest : public rclcpp::Node
{
public:
    CameraStreamTest() : Node("camera_stream_test")
    {
        // Initialize transport inside constructor without shared_from_this()
        image_transport_ = std::make_unique<image_transport::ImageTransport>(
            std::shared_ptr<rclcpp::Node>(this, [](auto*)
            {
            }));

        image_sub_ = image_transport_->subscribe(
            "/wrapper/psdk_ros2/compressed_camera_stream",
            5,
            [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg)
            {
                RCLCPP_INFO(get_logger(),
                            "Received image: %dx%d, encoding: %s, frame_id: %s",
                            msg->width, msg->height, msg->encoding.c_str(),
                            msg->header.frame_id.c_str());

                // Convert ROS Image to OpenCV format
                try
                {
                    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

                    // Display the image
                    cv::imshow("Camera Stream", cv_ptr->image);
                    cv::waitKey(1); // Process GUI events, wait 1ms
                }
                catch (cv_bridge::Exception& e)
                {
                    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
                }
            });

        RCLCPP_INFO(get_logger(), "Waiting for images...");
    }

    ~CameraStreamTest()
    {
        // Close all OpenCV windows when node is destroyed
        cv::destroyAllWindows();
    }

private:
    std::unique_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Subscriber image_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraStreamTest>();
    node->declare_parameter<std::string>("image_transport", "compressed");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
