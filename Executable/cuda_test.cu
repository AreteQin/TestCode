#include <iostream>
#include <cuda_runtime.h>

inline const char *_ConvertSMVer2ArchName(int major, int minor) {
    // Defines for GPU Architecture types (using the SM version to determine
    // the GPU Arch name)
    typedef struct {
        int SM;  // 0xMm (hexidecimal notation), M = SM Major version,
        // and m = SM minor version
        const char *name;
    } sSMtoArchName;

    sSMtoArchName nGpuArchNameSM[] = {
            {0x30, "Kepler"},
            {0x32, "Kepler"},
            {0x35, "Kepler"},
            {0x37, "Kepler"},
            {0x50, "Maxwell"},
            {0x52, "Maxwell"},
            {0x53, "Maxwell"},
            {0x60, "Pascal"},
            {0x61, "Pascal"},
            {0x62, "Pascal"},
            {0x70, "Volta"},
            {0x72, "Xavier"},
            {0x75, "Turing"},
            {0x80, "Ampere"},
            {0x86, "Ampere"},
            {-1,   "Graphics Device"}};

    int index = 0;

    while (nGpuArchNameSM[index].SM != -1) {
        if (nGpuArchNameSM[index].SM == ((major << 4) + minor)) {
            return nGpuArchNameSM[index].name;
        }

        index++;
    }

    // If we don't find the values, we default use the previous one
    // to run properly
    printf(
            "MapSMtoArchName for SM %d.%d is undefined."
            "  Default to use %s\n",
            major, minor, nGpuArchNameSM[index - 1].name);
    return nGpuArchNameSM[index - 1].name;
}
// end of GPU Architecture definitions

int main(void) {
    cudaDeviceProp devProp{};
    cudaGetDeviceProperties(&devProp, 0);
    std::cout << "使用GPU device " << 0 << ": " << devProp.name << std::endl;
    std::cout << "GPU Architecture: " << _ConvertSMVer2ArchName(devProp.major, devProp.minor) << std::endl;
    std::cout << "SM的数量：" << devProp.multiProcessorCount << std::endl;
    std::cout << "SM算力：" << devProp.major << "." << devProp.minor << std::endl;
    std::cout << "每个线程块的共享内存大小：" << devProp.sharedMemPerBlock / 1024.0 << " KB" << std::endl;
    std::cout << "每个线程块的最大线程数：" << devProp.maxThreadsPerBlock << std::endl;
    std::cout << "每个EM的最大线程数：" << devProp.maxThreadsPerMultiProcessor << std::endl;
    std::cout << "每个SM的最大线程束数：" << devProp.maxThreadsPerMultiProcessor / 32 << std::endl;
    return 0;
}