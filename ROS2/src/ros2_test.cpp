// rosbag_multi_extractor.cpp
//
// Single‑pass extractor that pulls images, Detection2DArray bounding boxes, and
// NavSatFix GPS messages from a ROS 2 bag and writes them to PNG/CSV files.
//
// Build (inside a ROS 2 workspace):
//     colcon build --packages-select <your_package> \
//         --cmake-args -DCMAKE_CXX_STANDARD=17
//
// Example usage:
//     rosbag_multi_extractor \
//         /home/qin/Downloads/m300_zigzag_images_GPS_box.mcap \
//         --image  /dji_osdk_ros/main_wide_RGB images_dir \
//         --bboxes /bounding_boxes/fire_spots  m300_box.csv \
//         --gps    /dji_osdk_ros/gps_position  m300_gps.csv
//
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_storage/storage_options.hpp"

// Image messages
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"

// Detection2DArray
#include "vision_msgs/msg/detection2_d_array.hpp"

// GPS NavSatFix
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace fs = std::filesystem;

// =========================================================================
// Helper structures for each extraction type
// =========================================================================
struct ImageOutput
{
    bool active{false};
    std::string topic;
    fs::path out_dir;
    std::ofstream csv;
    rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    std::uint64_t count{0};

    void open()
    {
        if (!active) return;
        if (!fs::exists(out_dir)) fs::create_directories(out_dir);
        fs::path csv_path = out_dir / "timestamps.csv";
        csv.open(csv_path);
        if (!csv) throw std::runtime_error("Cannot open " + csv_path.string());
        csv << "#timestamp_ns,filename\n";
    }

    void close() { if (csv.is_open()) csv.close(); }

    void process(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& msg)
    {
        if (!active || msg->topic_name != topic) return;
        rclcpp::SerializedMessage smsg(*msg->serialized_data);
        auto img = std::make_shared<sensor_msgs::msg::Image>();
        serializer.deserialize_message(&smsg, img.get());

        // std::uint64_t ts = static_cast<std::uint64_t>(img->header.stamp.sec) * 1'000'000'000ull + img->header.stamp.nanosec;
        std::uint64_t ts = rclcpp::Time(img->header.stamp).nanoseconds();

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            using sensor_msgs::image_encodings::BGR8;
            using sensor_msgs::image_encodings::MONO8;
            if (sensor_msgs::image_encodings::isColor(img->encoding))
            {
                cv_ptr = cv_bridge::toCvCopy(img, BGR8);
            }
            else if (sensor_msgs::image_encodings::isMono(img->encoding))
            {
                cv_ptr = cv_bridge::toCvCopy(img, MONO8);
            }
            else if (img->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || img->encoding ==
                sensor_msgs::image_encodings::MONO16)
            {
                cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_16UC1);
            }
            else
            {
                cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "cv_bridge error: " << e.what() << '\n';
            return;
        }

        if (!cv_ptr || cv_ptr->image.empty()) return;

        fs::path file = out_dir / (std::to_string(ts) + ".png");
        try
        {
            std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 3};
            if (!cv::imwrite(file.string(), cv_ptr->image, params)) throw std::runtime_error("imwrite failed");
        }
        catch (const cv::Exception& e)
        {
            std::cerr << "OpenCV error: " << e.what() << '\n';
            return;
        }

        csv << ts << ',' << file.filename().string() << '\n';
        if (++count % 100 == 0) std::cout << "Images: " << count << '\n';
    }
};

struct BBoxOutput
{
    bool active{false};
    std::string topic;
    std::ofstream csv;
    rclcpp::Serialization<vision_msgs::msg::Detection2DArray> serializer;

    void open(const std::string& path)
    {
        if (!active) return;
        csv.open(path);
        if (!csv) throw std::runtime_error("Cannot open " + path);
        csv << "#timestamp_ns,detection_idx,center_x,center_y,width,height,class,score\n";
    }

    void close() { if (csv.is_open()) csv.close(); }

    void process(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& msg)
    {
        if (!active || msg->topic_name != topic) return;
        rclcpp::SerializedMessage smsg(*msg->serialized_data);
        auto arr = std::make_shared<vision_msgs::msg::Detection2DArray>();
        serializer.deserialize_message(&smsg, arr.get());
        // std::uint64_t ts = static_cast<std::uint64_t>(arr->header.stamp.sec) * 1'000'000'000ull + arr->header.stamp.nanosec;
        std::uint64_t ts = rclcpp::Time(arr->header.stamp).nanoseconds();
        for (std::size_t i = 0; i < arr->detections.size(); ++i)
        {
            const auto& d = arr->detections[i];
            std::string label = "unknown";
            double score = 0.0;
            if (!d.results.empty())
            {
                label = d.results[0].hypothesis.class_id;
                score = d.results[0].hypothesis.score;
            }
            csv << ts << ',' << i << ',' << d.bbox.center.position.x << ',' << d.bbox.center.position.y << ','
                << d.bbox.size_x << ',' << d.bbox.size_y << ',' << label << ',' << score << '\n';
        }
    }
};

struct GPSOutput
{
    bool active{false};
    std::string topic;
    std::ofstream csv;
    rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serializer;

    void open(const std::string& path)
    {
        if (!active) return;
        csv.open(path);
        if (!csv) throw std::runtime_error("Cannot open " + path);
        csv << "#timestamp_ns,status,latitude_deg,longitude_deg,altitude_m";
        for (int i = 0; i < 9; ++i) csv << ",cov" << i;
        csv << ",covariance_type\n";
    }

    void close() { if (csv.is_open()) csv.close(); }

    void process(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& msg)
    {
        if (!active || msg->topic_name != topic) return;
        rclcpp::SerializedMessage smsg(*msg->serialized_data);
        auto fix = std::make_shared<sensor_msgs::msg::NavSatFix>();
        serializer.deserialize_message(&smsg, fix.get());
        // std::uint64_t ts = static_cast<std::uint64_t>(fix->header.stamp.sec) * 1'000'000'000ull + fix->header.stamp.nanosec;
        std::uint64_t ts = rclcpp::Time(fix->header.stamp).nanoseconds();
        csv << ts << ',' << static_cast<int>(fix->status.status) << ',' << fix->latitude << ',' << fix->longitude << ','
            << fix->altitude;
        for (double c : fix->position_covariance) csv << ',' << c;
        csv << ',' << static_cast<int>(fix->position_covariance_type) << '\n';
    }
};

// =========================================================================
// CLI parsing
// =========================================================================
struct Args
{
    std::string bag;
    ImageOutput img;
    BBoxOutput bbox;
    GPSOutput gps;
};

bool parse(int argc, char** argv, Args& a)
{
    if (argc < 2) return false;
    a.bag = argv[1];
    int i = 2;
    auto next = [&]() -> std::string
    {
        if (i >= argc) throw std::runtime_error("Missing arg");
        return argv[i++];
    };
    while (i < argc)
    {
        std::string f = argv[i++];
        if (f == "--image")
        {
            a.img.active = true;
            a.img.topic = next();
            a.img.out_dir = next();
        }
        else if (f == "--bboxes")
        {
            a.bbox.active = true;
            a.bbox.topic = next();
            a.bbox.open(next());
        }
        else if (f == "--gps")
        {
            a.gps.active = true;
            a.gps.topic = next();
            a.gps.open(next());
        }
        else
        {
            throw std::runtime_error("Unknown flag " + f);
        }
    }
    if (!a.img.active && !a.bbox.active && !a.gps.active) throw std::runtime_error("No extraction specified");
    return true;
}

// =========================================================================
// Main
// =========================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    Args args;
    try
    {
        if (!parse(argc, argv, args))
        {
            std::cerr << "Usage: " << argv[0] << " <bag_path> [--image <topic> <out_dir>] [--bboxes <topic> <csv>] "
                "[--gps <topic> <csv>]\n";
            return 1;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Argument error: " << e.what() << '\n';
        return 1;
    }

    try
    {
        args.img.open();

        rosbag2_storage::StorageOptions sopt;
        sopt.uri = args.bag;
        rosbag2_cpp::ConverterOptions copt;
        copt.input_serialization_format = "cdr";
        copt.output_serialization_format = "cdr";
        auto reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
        reader->open(sopt, copt);

        while (reader->has_next())
        {
            auto msg = reader->read_next();
            args.img.process(msg);
            args.bbox.process(msg);
            args.gps.process(msg);
        }

        args.img.close();
        args.bbox.close();
        args.gps.close();
        std::cout << "Extraction complete." << '\n';
    }
    catch (const std::exception& e)
    {
        std::cerr << "Runtime error: " << e.what() << '\n';
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
