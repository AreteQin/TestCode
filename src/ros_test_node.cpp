#include <ros/ros.h>
#include <boost/thread.hpp>
#include <vector>
#include <cmath>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// Shared data structure and mutex
std::vector<double> shared_data_x;
std::vector<double> shared_data_y;
boost::mutex data_mutex;

// Function to handle ROS node
void rosNodeFunction() {
    // Initialize the ROS node
    int argc = 0;
    char **argv = nullptr;
    ros::init(argc, argv, "ros_matplotlibcpp_node");
    ros::NodeHandle nh;

    // Example ROS loop
    ros::Rate loop_rate(10); // 10 Hz
    double t = 0.0;
    while (ros::ok()) {
        // Generate some data
        double x = t;
        double y = std::sin(t);

        // Lock the mutex and update shared data
        {
            boost::mutex::scoped_lock lock(data_mutex);
            shared_data_x.push_back(x);
            shared_data_y.push_back(y);

            // Maintain sliding window of the last 50 points
            if (shared_data_x.size() > 50) {
                shared_data_x.erase(shared_data_x.begin());
                shared_data_y.erase(shared_data_y.begin());
            }
        }

        // Increment the time variable
        t += 0.1;

        ROS_INFO("Generated data: x = %.2f, y = %.2f", x, y);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// Function to handle matplotlibcpp visualization
void matplotlibFunction() {
    // Turn on interactive mode
    plt::ion();

    while (ros::ok()) {
        // Local copy of shared data
        std::vector<double> x, y;

        // Lock the mutex and copy shared data
        {
            boost::mutex::scoped_lock lock(data_mutex);
            x = shared_data_x;
            y = shared_data_y;
        }

        // Clear the previous plot and plot updated data
        plt::clf();
        plt::plot(x, y, "b-");
        plt::title("Live Data Visualization");
        plt::grid(true);

        // Display the plot and update
        plt::pause(0.1); // Pause for 100 milliseconds
    }

    // Turn off interactive mode
    plt::show();
}

int main(int argc, char **argv) {
    // Create threads for ROS and matplotlibcpp
    boost::thread rosThread(rosNodeFunction);
    boost::thread matplotlibThread(matplotlibFunction);

    // Wait for both threads to finish
    rosThread.join();
    matplotlibThread.join();

    return 0;
}
