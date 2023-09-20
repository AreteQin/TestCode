#include "iostream"
#include "math.h"
#include <opencv2/viz/widgets.hpp>
#include <string>
#include <glog/logging.h>
#include <fstream>

using namespace std;

class FileWritter {
public:
    FileWritter(const std::string file, const int number_precision)
            : fileName(file), num_precision(number_precision) {
        oufile.flags(std::ios::fixed);
        oufile.precision(num_precision);
    }
    ~FileWritter() {
        close();
        LOG(INFO)<<"file: "<<fileName.c_str()<<" is closed";
    }

    void open() { oufile.open(fileName.c_str(), std::ios::app | std::ios::out); }
    void close() { oufile.close(); }
//    void setDelimiter(const std::string del) { delimiter = del; }
    void erase() {
        /* NOTE: fstream has a constructor same as open, can open file when the
         * NOTE: instance!
         * */
        std::ofstream fileErase(fileName.c_str(), std::ios::out | std::ios::trunc);
        fileErase.close();
    }
    void new_open() {
        erase();
        open();
    }

    template <typename T, typename... Ts>
    void write(T arg1, Ts... arg_left) {
        oufile << arg1 << delimiter;
        write(arg_left...);
    }

private:
    std::string fileName;
    std::string delimiter{","};
    std::fstream oufile;
    int num_precision;

    void write() { oufile << std::endl; }
};

int main (){
    // VINS extrinsics for M300
//    double R21 = -0.923949;
//    double R11 = -0.148326;
//    double R31 = -0.352589;
//    double R32 = -0.9308;
//    double R33 = 0.096396;
// Kalibr extrinsics for QCar

// VINS extrinsics for QCar
//    0.0157658 0.0376125  0.999168
//                         -0.993578  0.112572   0.01144
//                                               -0.112049 -0.992931 0.0391457
// 0.000133774 -0.00115319  -0.0997888
//    double R21 = -0.01499097;
//    double R11 = -0.01730286;
//    double R31 = 0.99973791;
//    double R32 = -0.01730136;
//    double R33 = -0.01499269;
//    double roll = atan2(R32, R33);
//    double pitch = atan2(-R31, sqrt(R32 * R32 + R33 * R33));
//    double yaw = atan2(R21, R11);
//    cout << "roll: " << roll << endl;
//    cout << "pitch: " << pitch << endl;
//    cout << "yaw: " << yaw << endl;


    // Initialize Google's logging library.
    google::InitGoogleLogging("test");
    FLAGS_logtostderr = true; // log to stderr instead of log files

    // use file writter to write data to csv file
    FileWritter fileWritter("m300_ref_GPS_path.csv", 8);
    fileWritter.open();
    fileWritter.write(1, 2, 3, 4, 5, 6);
    // erease the file
    fileWritter.erase();
    fileWritter.close();

    return 0;
}