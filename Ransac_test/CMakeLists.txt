cmake_minimum_required(VERSION 3.10)

set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

set(CMAKE_C_FLAGS "-std=gnu99 -W -Wno-attribute-warning -fopenmp -O2 -fPIC ")
set(CMAKE_CXX_FLAGS " -W -Wno-attribute-warning -fopenmp -O2 -fPIC ")

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set the project name and version
project(Test_RANSAC VERSION 1)

# include crc
INCLUDE_DIRECTORIES(~/Documents/cpp-projects/matplotlib-cpp-master)
INCLUDE_DIRECTORIES(../Sources)
INCLUDE_DIRECTORIES(../Headers)

# add the executable
add_executable(Test ransac_test.cpp Sources/lateralestimator.cpp Headers/lateralestimator.h matplotlibcpp.h)

# include matplotlib
find_package(Python3 COMPONENTS Development)
target_include_directories(Test_RANSAC PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
target_link_libraries(myproject Python2::Python)

# link boost library
TARGET_LINK_LIBRARIES( Test  LINK_PUBLIC -lpthread -ltbb -ldl -lutil -lm)