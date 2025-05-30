cmake_minimum_required(VERSION 3.8)
#project(Test)
project(Test LANGUAGES CXX CUDA)

#set(CMAKE_BUILD_TYPE "debug")
#add_definitions("-DENABLE_SSE")
#set(CMAKE_CXX_FLAGS "-std=c++17 ${SSE_FLAGS} -g -O3 -march=native")
#set(CMAKE_CXX_FLAGS "-std=c++17 -O3 -fopenmp") # -fopenmp enable multi-thread
#set(CMAKE_CXX_FLAGS "-std=c++17")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules) # Add elements to list
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4 REQUIRED)
find_package(Sophus REQUIRED)
#find_package(Pangolin REQUIRED) # used to show pointcloud
#find_package(CSparse REQUIRED)

find_package(G2O REQUIRED)
if (G2O_FOUND)
    message(STATUS "G2O found. Version: ${G2O_VERSION}")
endif (G2O_FOUND)

find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
#find_package(realsense2 REQUIRED)
#find_package(GLFW REQUIRED)
find_package(Boost COMPONENTS system filesystem serialization REQUIRED)
#find_package(OpenMP REQUIRED)
#find_package(Ceres REQUIRED)

#find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#add_definitions(${PCL_DEFINITIONS})

find_package(assimp REQUIRED)
if (assimp_FOUND)
    set(assimp_INCLUDE_DIRS /usr/include/assimp)
    set(assimp_LIBRARIES /usr/lib/x86_64-linux-gnu/libassimp.so)
    message(STATUS "Assimp found at ${assimp_INCLUDE_DIRS}")
    include_directories(${assimp_INCLUDE_DIRS})
else ()
    message(FATAL_ERROR "Assimp not found")
endif ()

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
set(CMAKE_AUTOMOC ON)

find_package(CUDA REQUIRED)
#if (CUDA_FOUND)
#    message("-- Found CUDA version ${CUDA_VERSION} installed in: "
#            "${CUDA_TOOLKIT_ROOT_DIR} via legacy (< 3.17) CMake module. "
#            "Using the legacy CMake module means that any installation of "
#            "Ceres will require that the CUDA libraries be installed in a "
#            "location included in the LD_LIBRARY_PATH.")
#    enable_language(CUDA)
#
#    macro(DECLARE_IMPORTED_CUDA_TARGET COMPONENT)
#        add_library(CUDA::${COMPONENT} INTERFACE IMPORTED)
#        target_include_directories(
#                CUDA::${COMPONENT} INTERFACE ${CUDA_INCLUDE_DIRS})
#        target_link_libraries(
#                CUDA::${COMPONENT} INTERFACE ${CUDA_${COMPONENT}_LIBRARY} ${ARGN})
#    endmacro()
#
#    declare_imported_cuda_target(cublas)
#    declare_imported_cuda_target(cusolver)
#    declare_imported_cuda_target(cusparse)
#    declare_imported_cuda_target(cudart ${CUDA_LIBRARIES})
#
#    set(CUDAToolkit_BIN_DIR ${CUDA_TOOLKIT_ROOT_DIR}/bin)
#else (CUDA_FOUND)
#    message("-- Did not find CUDA, disabling CUDA support.")
#    update_cache_variable(USE_CUDA OFF)
#endif (CUDA_FOUND)
set(CUDA_ADD_CUBLAS_TO_TARGET ON)

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
        ${Pangolin_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
        ${REALSENSE_INCLUDE_DIR}
        #        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
        #        ${GLFW_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        #        ${CERES_INCLUDE_DIRS}
)

# enable drawing plot
add_subdirectory(ThirdParty/matplotlib-cpp)
include_directories(${PROJECT_SOURCE_DIR}/third_party/matplotlib-cpp)
# Find Python headers
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

#add_library(my_lib lib.cpp lib.h)
add_executable(testing_codes test.cpp) # "name "test" is not valid in TX2
#target_include_directories(test PUBLIC ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES} ${GLOG_INCLUDE_DIRS})
target_link_libraries(testing_codes
        Eigen3::Eigen
        matplotlib_cpp
        ${realsense2_LIBRARY}
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${PCL_LIBRARIES}
        g2o_core
        g2o_stuff
        g2o_types_sba
        g2o_solver_csparse
        g2o_csparse_extension
        g2o_types_sim3
        Sophus::Sophus
        Qt5::Widgets
        Qt5::Core
        Qt5::Gui
)

add_executable(cuda_test cuda_test.cu cuda_test.cuh)
target_include_directories(cuda_test PUBLIC
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
)