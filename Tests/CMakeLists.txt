cmake_minimum_required(VERSION 3.10)

set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set the project name and version
project(Test_CRC VERSION 1)

# include boost
set(Boost_LIBRARIES, "usr/include/boost")
INCLUDE_DIRECTORIES(${Boost_LIBRARIES})

# add the executable
add_executable(Test Test.cpp)

# link boost library
TARGET_LINK_LIBRARIES( Main LINK_PUBLIC ${Boost_LIBRARIES} -lboost_system -lboost_regex -lboost_thread -lpthread -ltbb -ldl -lutil -lm)






