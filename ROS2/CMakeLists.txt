cmake_minimum_required(VERSION 3.5)
project(ros2_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add executable
add_executable(ros2_test src/ros2_test.cpp) # Replace src/my_node.cpp with your source file

# Link dependencies
ament_target_dependencies(ros2_test rclcpp std_msgs) # Add other dependencies as needed

# Install the executable (optional, but recommended)
install(TARGETS
        ros2_test
        DESTINATION lib/${PROJECT_NAME}
)

# Install the include directory (if you have header files)
# install(DIRECTORY include/${PROJECT_NAME}
#   DESTINATION include/${PROJECT_NAME}
# )

ament_package()
