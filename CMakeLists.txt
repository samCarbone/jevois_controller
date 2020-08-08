cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(Conductor VERSION 0.1)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# add the sources subdirectory
add_subdirectory(Sources)

# add include directories
target_include_directories(Conductor PUBLIC "${PROJECT_BINARY_DIR}/Headers")

# add the executable
add_executable(Main main.cpp)

# CMake configuration inherited from jevois
set(JEVOIS_PLATFORM_C_COMPILER, "/usr/share/jevois-sdk/out/sun8iw5p1/linux/common/buildroot/host/usr/bin/arm-buildroot-linux-gnueabihf-gcc")
set(JEVOIS_PLATFORM_CXX_COMPILER, "/usr/share/jevois-sdk/out/sun8iw5p1/linux/common/buildroot/host/usr/bin/arm-buildroot-linux-gnueabihf-g++")

