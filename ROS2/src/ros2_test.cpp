// src/image_bag_to_folder.cpp

#include <fstream>
#include <memory>
#include <string>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_cpp/reader.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

class ImageBagToFolder : public rclcpp::Node
{
public:
  ImageBagToFolder(
    const std::string & bag_path,
    const std::string & topic_name,
    const std::string & out_folder)
  : Node("image_bag_to_folder")
  {
    // Create output directory (and parent dirs) if needed
    fs::create_directories(out_folder);

    // Open timestamps file
    std::string ts_file = (fs::path(out_folder) / "timestamps.txt").string();
    ts_stream_.open(ts_file, std::ios::out);
    if (!ts_stream_.is_open()) {
      throw std::runtime_error("Could not open " + ts_file);
    }
    ts_stream_ << "#timestamp [ns],filename\n";

    // Initialize rosbag2 reader
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;

    reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
    reader_->open(storage_options);

    topic_name_ = topic_name;
    out_folder_ = out_folder;
  }

  void run()
  {
    rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    size_t frame_idx = 0u;

    while (reader_->has_next()) {
      auto bag_msg = reader_->read_next();
      if (bag_msg->topic_name != topic_name_) {
        continue;
      }

      // Deserialize ROS2 Image
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
      serializer.deserialize_message(&serialized, img_msg.get());

      // Convert to OpenCV
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        continue;
      }

      // Timestamp in ns
      uint64_t ts_ns =
        static_cast<uint64_t>(img_msg->header.stamp.sec) * 1000000000ull +
        img_msg->header.stamp.nanosec;

      // Build filename
      char buf[64];
      std::snprintf(buf, sizeof(buf), "frame_%06zu.png", frame_idx);
      std::string filename = buf;
      std::string fullpath = (fs::path(out_folder_) / filename).string();

      // Save image
      if (!cv::imwrite(fullpath, cv_ptr->image)) {
        RCLCPP_WARN(get_logger(), "Failed to write %s", fullpath.c_str());
        continue;
      }

      // Log timestamp
      ts_stream_ << ts_ns << "," << filename << "\n";

      ++frame_idx;
    }

    ts_stream_.close();
    RCLCPP_INFO(get_logger(), "Done: wrote %zu images to %s", frame_idx, out_folder_.c_str());
  }

private:
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  std::string topic_name_, out_folder_;
  std::ofstream ts_stream_;
};

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0]
              << " <bag_path> <image_topic> <output_folder>\n";
    return 1;
  }
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<ImageBagToFolder>(
      argv[1],  // path to bag (folder)
      argv[2],  // e.g. "/camera/image_raw"
      argv[3]); // e.g. "out_images"
    node->run();
  } catch (const std::exception & ex) {
    std::cerr << "Error: " << ex.what() << "\n";
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
