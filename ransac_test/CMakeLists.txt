cmake_minimum_required(VERSION 3.10)

set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

set(CMAKE_C_FLAGS "-std=gnu99 -W -fopenmp -O2 -fPIC ")
set(CMAKE_CXX_FLAGS " -W -fopenmp -O2 -fPIC ")

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set the project name and version
project(Test VERSION 1)

INCLUDE_DIRECTORIES(~/Documents/cpp-playground/matplotlib-cpp)
INCLUDE_DIRECTORIES(../Sources)
INCLUDE_DIRECTORIES(../Headers)

# include matplotlib
find_package(Python COMPONENTS Development NumPy)

# add the executable
add_executable(Test ransac_test.cpp ../Sources/lateralestimator.cpp ../Headers/lateralestimator.h ../../matplotlib-cpp/matplotlibcpp.h)
target_include_directories(Test PRIVATE ${Python_INCLUDE_DIRS} ${Python_NumPy_INCLUDE_DIRS})


# link library
TARGET_LINK_LIBRARIES( Test  LINK_PUBLIC -lpthread -ltbb -ldl -lutil -lm Python::Python Python::NumPy)
