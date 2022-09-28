#include <librealsense2/rs.hpp> // Include Intel RealSense Cross Platform API
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;

int main(){
    rs2::colorizer color_map;
    // Declare the RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with the default recommended configuration
    pipe.start();
    while(1){
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_depth_frame(); // Find and colorize the depth data
        rs2::frame color = data.get_color_frame();            // Find the color data
        // Render depth on to the first half of the screen and color on to the second
        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();
        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat depth_mat(Size(w, h), CV_16UC1, (void*)depth.get_data(), Mat::AUTO_STEP);
        Mat color_mat(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        cvtColor(color_mat, color_mat, COLOR_RGB2BGR);
        imshow("depth", depth_mat);
        imshow("color", color_mat);
        waitKey(1);
    }
    return 0;
}