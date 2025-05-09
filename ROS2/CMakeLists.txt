cmake_minimum_required(VERSION 3.5)
project(ros2_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(vision_msgs REQUIRED)

find_package(Glog REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

# Add executable
add_executable(ros2_test src/ros2_test.cpp) # Replace src/my_node.cpp with your source file

# Link dependencies
ament_target_dependencies(ros2_test
        rclcpp
        std_msgs
        "rclcpp_lifecycle"
        "lifecycle_msgs"
        cv_bridge
        sensor_msgs
        image_transport
        rosbag2_transport
        rosbag2_cpp
        compressed_image_transport
        vision_msgs
) # Add other dependencies as needed

# Link OpenCV
target_link_libraries(ros2_test
        glog::glog
        ${OpenCV_LIBS})

# Install the executable (optional, but recommended)
install(TARGETS
        ros2_test
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)

# Install the include directory (if you have header files)
# install(DIRECTORY include/${PROJECT_NAME}
#   DESTINATION include/${PROJECT_NAME}
# )

ament_package()
