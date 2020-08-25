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

# include crc
# INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR}/../CRCpp/inc/)
# set(CRC_DIRECTORY, ${PROJECT_SOURCE_DIR}/../CRCpp/inc/)
# MESSAGE(STATUS "${CRC_DIRECTORY}")

# INCLUDE_DIRECTORIES(${CRC_DIRECTORY})
# INCLUDE_DIRECTORIES(/Users/samuelcarbone/Documents/cpp-projects/jevois_controller/CRCpp/inc/)
INCLUDE_DIRECTORIES(~/Documents/cpp-playground/jevois_controller/CRCpp/inc/)

# add the executable
add_executable(Test test.cpp)

# link boost library
TARGET_LINK_LIBRARIES( Test  LINK_PUBLIC ${Boost_LIBRARIES} -lboost_system -lboost_regex -lboost_thread -lpthread -ltbb -ldl -lutil -lm)






