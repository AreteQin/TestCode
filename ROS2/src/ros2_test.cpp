#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <iomanip> // For std::setprecision
#include <filesystem> // Requires C++17

// Rosbag2
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_storage/storage_filter.hpp"

// ROS 2 messages and types
#include "rclcpp/rclcpp.hpp" // Minimal initialization if needed, and for serialization
#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/image.hpp"

// OpenCV and cv_bridge
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp" // For cv::imwrite
#include "sensor_msgs/image_encodings.hpp" // For encoding constants

namespace fs = std::filesystem;

int main(int argc, char *argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <path_to_bag_file> <output_directory> <image_topic>" << std::endl;
        std::cerr << "Example: " << argv[0] << " /path/to/my_bag output_images /camera/image_raw" << std::endl;
        return 1;
    }

    const std::string bag_path_str = argv[1];
    const std::string output_dir_str = argv[2];
    const std::string image_topic = argv[3];

    fs::path bag_path(bag_path_str);
    fs::path output_dir_path(output_dir_str);
    fs::path csv_file_path = output_dir_path / "timestamps.csv";

    // --- 1. Setup Output Directory and CSV ---
    try {
        if (!fs::exists(output_dir_path)) {
            if (!fs::create_directories(output_dir_path)) {
                std::cerr << "Error: Could not create output directory: " << output_dir_path << std::endl;
                return 1;
            }
            std::cout << "Created output directory: " << output_dir_path << std::endl;
        } else if (!fs::is_directory(output_dir_path)) {
             std::cerr << "Error: Output path exists but is not a directory: " << output_dir_path << std::endl;
             return 1;
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
        return 1;
    }

    std::ofstream csv_file(csv_file_path);
    if (!csv_file.is_open()) {
        std::cerr << "Error: Could not open CSV file for writing: " << csv_file_path << std::endl;
        return 1;
    }
    // Write CSV header
    csv_file << "#timestamp [ns],filename" << std::endl;
    std::cout << "Opened CSV file for writing: " << csv_file_path << std::endl;

    // --- 2. Setup Rosbag Reader ---
    rosbag2_storage::StorageOptions storage_options{};
    storage_options.uri = bag_path_str; // Use the string version
    storage_options.storage_id = "sqlite3"; // Or "mcap" if your bag uses that format

    rosbag2_cpp::ConverterOptions converter_options{};
    converter_options.input_serialization_format = "cdr"; // Common format, adjust if needed
    converter_options.output_serialization_format = "cdr";

    std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader =
        std::make_unique<rosbag2_cpp::readers::SequentialReader>();

    try {
         reader->open(storage_options, converter_options);
    } catch (const std::exception & e) {
        std::cerr << "Error opening bag file: " << e.what() << std::endl;
        csv_file.close();
        return 1;
    }

    // --- 3. Filter for the specified image topic ---
    if (!image_topic.empty()) {
        rosbag2_storage::StorageFilter filter;
        filter.topics = {image_topic};
        reader->set_filter(filter);
        std::cout << "Filtering for topic: " << image_topic << std::endl;
    } else {
        std::cerr << "Warning: No image topic specified. Processing all topics (expect errors if non-image topics found)." << std::endl;
    }


    // --- 4. Prepare for Message Deserialization and Image Conversion ---
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;
    int image_count = 0;

    std::cout << "Starting image extraction..." << std::endl;

    // --- 5. Read Messages and Process Images ---
    while (reader->has_next()) {
        try {
            auto bag_message = reader->read_next();

            // Check if the topic matches (redundant if filter is set, but good practice)
            if (bag_message->topic_name != image_topic) {
                continue;
            }

            // Get timestamp (nanoseconds)
            rclcpp::Time msg_time(bag_message->time_stamp);
            int64_t timestamp_ns = msg_time.nanoseconds();

            // Deserialize the message data
            auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            image_serialization.deserialize_message(&serialized_msg, image_msg.get());

            // Convert ROS Image message to OpenCV Mat
            // Use toCvCopy to ensure we have our own copy of the image data
            // Try to convert to BGR8, which is common for saving with OpenCV
            cv_bridge::CvImagePtr cv_ptr;
            try {
                 // Common encodings - try to convert to something OpenCV understands well like BGR8 or MONO8
                if (sensor_msgs::image_encodings::isColor(image_msg->encoding) ||
                    image_msg->encoding == sensor_msgs::image_encodings::BAYER_RGGB8 || // Add other Bayer types if needed
                    image_msg->encoding == sensor_msgs::image_encodings::BAYER_BGGR8 ||
                    image_msg->encoding == sensor_msgs::image_encodings::BAYER_GBRG8 ||
                    image_msg->encoding == sensor_msgs::image_encodings::BAYER_GRBG8)
                {
                     cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
                }
                else if (sensor_msgs::image_encodings::isMono(image_msg->encoding))
                {
                     cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
                }
                 else if (image_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || // Handle depth images (save as 16-bit PNG)
                          image_msg->encoding == sensor_msgs::image_encodings::MONO16) // depth16 is an alias often used
                 {
                      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
                 }
                else {
                    std::cerr << "Warning: Unhandled image encoding '" << image_msg->encoding
                              << "' for timestamp " << timestamp_ns << ". Attempting direct copy." << std::endl;
                    // Fallback: try to copy directly, imwrite might fail
                    cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
                }

            } catch (const cv_bridge::Exception& e) {
                std::cerr << "cv_bridge exception for timestamp " << timestamp_ns
                          << ": " << e.what() << " (Original encoding: " << image_msg->encoding << ")" << std::endl;
                continue; // Skip this message
            } catch (const std::runtime_error& e) { // Catch potential errors if encoding field is invalid
                 std::cerr << "Runtime error during cv_bridge conversion for timestamp " << timestamp_ns
                           << ": " << e.what() << " (Original encoding: " << image_msg->encoding << ")" << std::endl;
                 continue;
            }


            if (!cv_ptr || cv_ptr->image.empty()) {
                std::cerr << "Error: Failed to convert image or image is empty for timestamp " << timestamp_ns << std::endl;
                continue;
            }

            // Construct filename and full path
            std::string image_filename = std::to_string(timestamp_ns) + ".png";
            fs::path output_image_path = output_dir_path / image_filename;

            // Save the image as PNG
             std::vector<int> compression_params;
             compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
             compression_params.push_back(3); // Default compression level (0-9, higher is more compression)

            bool success = false;
             try {
                 success = cv::imwrite(output_image_path.string(), cv_ptr->image, compression_params);
             } catch (const cv::Exception& e) {
                 std::cerr << "OpenCV exception during imwrite for " << output_image_path.string() << ": " << e.what() << std::endl;
                 continue; // Skip writing to CSV if image saving failed
             }


            if (!success) {
                std::cerr << "Error: Failed to write image file: " << output_image_path << std::endl;
                continue; // Skip writing to CSV if image saving failed
            }

            // Write timestamp and filename to CSV
            // Use fixed and setprecision for consistent timestamp output if needed, but nanoseconds are integers
            csv_file << timestamp_ns << "," << image_filename << std::endl;
            image_count++;

            if (image_count % 100 == 0) {
                 std::cout << "Processed " << image_count << " images..." << std::endl;
            }


        } catch (const std::exception& e) {
            std::cerr << "Error processing message: " << e.what() << std::endl;
            // Decide whether to continue or stop on error
            // continue;
        }
    }

    // --- 6. Cleanup ---
    csv_file.close();
    // reader->close(); // reader is RAII via unique_ptr, no explicit close needed unless specific cleanup required

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Image extraction finished." << std::endl;
    std::cout << "Total images extracted: " << image_count << std::endl;
    std::cout << "Output directory: " << output_dir_path.string() << std::endl;
    std::cout << "Timestamps CSV: " << csv_file_path.string() << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    return 0;
}