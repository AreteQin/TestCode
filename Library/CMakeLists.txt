cmake_minimum_required(VERSION 3.8)
project(test_library)

add_library(my_lib lib.cpp lib.h)

# install the library by running `make install`
install(TARGETS my_lib DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/lib)
message("Run \"make install\" to install the library to ${CMAKE_CURRENT_SOURCE_DIR}/lib")