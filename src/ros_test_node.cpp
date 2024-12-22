#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Header.h>
#include <glog/logging.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <vector>
#include <string>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>

// Path to KITTI data
const std::string DATA_PATH = "/home/qin/Downloads/2011_09_26/2011_09_26_drive_0002_sync/velodyne_points/data/";

// Function to read binary point cloud data from KITTI file
std::vector<float> readPointCloudData(const std::string& filepath)
{
    std::ifstream file(filepath, std::ios::binary);
    if (!file)
    {
        LOG(ERROR) << "Unable to open file: " << filepath;
        return {};
    }

    std::vector<float> points;
    file.seekg(0, std::ios::end);
    size_t fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    points.resize(fileSize / sizeof(float));
    file.read(reinterpret_cast<char*>(points.data()), fileSize);
    return points;
}

class KittiPointCloudNode
{
public:
    KittiPointCloudNode(const std::string& data_path, int frame_limit, int publish_rate)
        : data_path_(data_path), frame_limit_(frame_limit), frame_(0), nh_("~"), rate_(publish_rate)
    {
        pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("kitti/pointcloud", 10);
    }

    void spin()
    {
        while (ros::ok())
        {
            publishPointCloud();
            ros::spinOnce();
            rate_.sleep();
        }
    }

private:
    void publishPointCloud()
    {
        // Construct the file path for the current frame
        char filepath[512];
        snprintf(filepath, sizeof(filepath), "%s%010d.bin", DATA_PATH.c_str(), frame_);

        // Read point cloud data
        std::vector<float> point_data = readPointCloudData(filepath);
        if (point_data.empty())
        {
            LOG(ERROR) << "Failed to read point cloud data from file: " << filepath;
            frame_ = (frame_ + 1) % frame_limit_;
            rate_.sleep();
            return;
        }

        size_t num_points = point_data.size() / 4; // Each point has 4 values: x, y, z, intensity

        // Create a PointCloud2 message
        sensor_msgs::PointCloud2 pcl_msg;
        pcl_msg.header.stamp = ros::Time::now();
        pcl_msg.header.frame_id = "map";
        pcl_msg.height = 1;
        pcl_msg.width = num_points;
        pcl_msg.is_dense = false;
        pcl_msg.is_bigendian = false;

        // Define fields: x, y, z, intensity
        sensor_msgs::PointField field_x, field_y, field_z, field_i;
        field_x.name = "x";
        field_x.offset = 0;
        field_x.datatype = sensor_msgs::PointField::FLOAT32;
        field_x.count = 1;
        field_y.name = "y";
        field_y.offset = 4;
        field_y.datatype = sensor_msgs::PointField::FLOAT32;
        field_y.count = 1;
        field_z.name = "z";
        field_z.offset = 8;
        field_z.datatype = sensor_msgs::PointField::FLOAT32;
        field_z.count = 1;
        field_i.name = "i";
        field_i.offset = 12;
        field_i.datatype = sensor_msgs::PointField::FLOAT32;
        field_i.count = 1;

        pcl_msg.fields = {field_x, field_y, field_z, field_i};
        pcl_msg.point_step = 16; // 4 fields * 4 bytes (float)
        pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;

        // Fill the data
        pcl_msg.data.resize(pcl_msg.row_step);
        memcpy(pcl_msg.data.data(), point_data.data(), pcl_msg.row_step);

        // Publish the point cloud
        LOG(INFO) << "Publishing point cloud with " << num_points << " points";
        pcl_pub_.publish(pcl_msg);

        // Increment frame
        frame_ = (frame_ + 1) % frame_limit_;
    }

    std::vector<float> readPointCloudData(const std::string& filepath)
    {
        std::ifstream file(filepath, std::ios::binary);
        if (!file)
        {
            LOG(ERROR) << "Unable to open file: " << filepath;
            return {};
        }

        std::vector<float> points;
        file.seekg(0, std::ios::end);
        size_t fileSize = file.tellg();
        file.seekg(0, std::ios::beg);

        points.resize(fileSize / sizeof(float));
        file.read(reinterpret_cast<char*>(points.data()), fileSize);
        return points;
    }

    ros::NodeHandle nh_;
    ros::Publisher pcl_pub_;
    ros::Rate rate_;
    std::string data_path_;
    int frame_limit_;
    int frame_;
};

class KittiMarkerNode
{
public:
    KittiMarkerNode(const std::string& topic_name, int publish_rate, visualization_msgs::Marker marker)
        : nh_("~"), rate_(publish_rate), frame_(0), marker_(std::move(marker)), topic_name_(topic_name)
    {
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>(topic_name_, 10);
    }

    void spin()
    {
        while (ros::ok())
        {
            publishMarker();
            ros::spinOnce();
            rate_.sleep();
        }
    }

private:
    void publishMarker()
    {
        marker_.header.stamp = ros::Time::now();

        // marker.ns = "basic_shapes";
        marker_.lifetime = ros::Duration();

        marker_pub_.publish(marker_);
        LOG(INFO) << "Publishing marker";
    }

    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Rate rate_;
    int frame_;
    visualization_msgs::Marker marker_;
    std::string topic_name_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti_node");
    ros::NodeHandle nh;

    // Initialize point cloud publisher
    KittiPointCloudNode point_cloud_pub(DATA_PATH, 76, 10);

    visualization_msgs::Marker drone_model;
    drone_model.header.frame_id = "map";
    drone_model.header.stamp = ros::Time::now();
    drone_model.ns = "fbx_marker";
    drone_model.id = 0;
    drone_model.type = visualization_msgs::Marker::MESH_RESOURCE;
    if (std::ifstream("/home/qin/Downloads/TestCodes/dji_m30_model.dae")) {
        drone_model.mesh_resource = "file:///home/qin/Downloads/TestCodes/dji_m30_model.dae";
    } else {
        LOG(ERROR) << "Mesh resource file not found: /home/qin/Downloads/TestCodes/dji_m30_model.dae";
    }
    drone_model.action = visualization_msgs::Marker::ADD;
    drone_model.pose.position.x = -2.0;
    drone_model.pose.position.y = 0.0;
    drone_model.pose.position.z = 10.0;
    // drone_model.pose.orientation.x = 0.0;
    // drone_model.pose.orientation.y = 0.0;
    // drone_model.pose.orientation.z = 1.0;
    // drone_model.pose.orientation.w = 1.0;
    drone_model.color.r = 1.0;
    drone_model.color.g = 1.0;
    drone_model.color.b = 1.0;
    drone_model.color.a = 1.0;
    drone_model.scale.x = 0.02;
    drone_model.scale.y = 0.02;
    drone_model.scale.z = 0.02;
    // rotate the model
    // Set the rotation angle (90 degrees around the y-axis)
    double angle = M_PI / 2.0; // 90 degrees in radians
    // Create a quaternion representing the rotation
    tf2::Quaternion quaternion;
    quaternion.setRPY(angle, 0, angle); // Roll, Pitch, Yaw
    // Convert the quaternion to a geometry_msgs::Quaternion
    geometry_msgs::Quaternion q_msg;
    q_msg.x = quaternion.x();
    q_msg.y = quaternion.y();
    q_msg.z = quaternion.z();
    q_msg.w = quaternion.w();
    // Set the orientation of the drone model marker
    drone_model.pose.orientation = q_msg;

    visualization_msgs::Marker ego;
    ego.header.frame_id = "map";
    ego.header.stamp = ros::Time::now();
    ego.ns = "ego_marker";
    ego.id = 1;
    ego.type = visualization_msgs::Marker::LINE_STRIP;
    ego.action = visualization_msgs::Marker::ADD;
    ego.lifetime = ros::Duration();
    ego.pose.position.x = -2.0;
    ego.pose.position.y = 0.0;
    ego.pose.position.z = 10.0;
    // ego.pose.orientation.x = 0.0;
    // ego.pose.orientation.y = 0.0;
    // ego.pose.orientation.z = 1.0;
    // ego.pose.orientation.w = 1.0;
    ego.color.r = 1.0;
    ego.color.g = 0.0;
    ego.color.b = 0.0;
    ego.color.a = 1.0;
    ego.scale.x = 0.1;
    geometry_msgs::Point first_line_point;
    first_line_point.x = 1.0;
    first_line_point.y = -1.0;
    first_line_point.z = 0.0;
    geometry_msgs::Point second_line_point;
    second_line_point.x = 1.0;
    second_line_point.y = 1.0;
    second_line_point.z = 0.0;
    geometry_msgs::Point original_point;
    original_point.x = 0.0;
    original_point.y = 0.0;
    original_point.z = 0.0;
    ego.points.push_back(first_line_point);
    ego.points.push_back(original_point);
    ego.points.push_back(second_line_point);

    // Initialize marker publisher
    KittiMarkerNode drone_model_pub("drone",10, drone_model);
    KittiMarkerNode ego_pub("ego", 10, ego);

    // Run both publishers concurrently
    std::thread point_cloud_thread(&KittiPointCloudNode::spin, &point_cloud_pub);
    std::thread drone_model_thread(&KittiMarkerNode::spin, &drone_model_pub);
    std::thread ego_thread(&KittiMarkerNode::spin, &ego_pub);

    point_cloud_thread.join();
    drone_model_thread.join();
    ego_thread.join();

    return 0;
}
