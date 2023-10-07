#include "iostream"
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