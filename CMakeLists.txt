cmake_minimum_required(VERSION 3.0)
#project(Test)
project(Test LANGUAGES CXX CUDA)

set(CMAKE_BUILD_TYPE "debug")
#add_definitions("-DENABLE_SSE")
#set(CMAKE_CXX_FLAGS "-std=c++17 ${SSE_FLAGS} -g -O3 -march=native")
#set(CMAKE_CXX_FLAGS "-std=c++17 -O3 -fopenmp")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules) # Add elements to list
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4 REQUIRED)
find_package(Sophus REQUIRED)
#find_package(Pangolin REQUIRED) # used to show pointcloud
find_package(CSparse REQUIRED)
find_package(G2O REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(realsense2 REQUIRED)
#find_package(GLFW REQUIRED)
#find_package(Boost COMPONENTS system filesystem serialization REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Ceres REQUIRED)

if (OPENMP_FOUND)
    message(STATUS "OpenMP found will try to link!")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
#        "/usr/include/eigen3/"
        ${Pangolin_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
#        ${REALSENSE_INCLUDE_DIR}
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
#        ${GLOG_INCLUDE_DIRS}
#        ${GLFW_INCLUDE_DIRS}
#        ${Boost_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
)

# enable drawing plot
#add_subdirectory(third_party/matplotlib-cpp)

add_library(my_lib lib.cpp lib.h)
add_executable(testing_codes test.cpp cuda_test.cuh) # "name "test" is not valid in TX2
#target_include_directories(test PUBLIC ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES} ${GLOG_INCLUDE_DIRS})
target_link_libraries(testing_codes
        my_lib
#        matplotlib_cpp
        ${realsense2_LIBRARY}
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        ${GLOG_LIBRARIES}
#        ${GLFW_LIBRARIES}
        g2o_core
        g2o_stuff
        g2o_types_sba
        g2o_solver_csparse
        g2o_csparse_extension
        Sophus::Sophus
        -lboost_serialization
        ${CERES_LIBRARIES}
        )
