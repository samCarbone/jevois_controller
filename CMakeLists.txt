cmake_minimum_required(VERSION 3.10)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# CMake configuration inherited from jevois
# set(JEVOIS_PLATFORM_C_COMPILER, "/usr/share/jevois-sdk/out/sun8iw5p1/linux/common/buildroot/host/usr/bin/arm-buildroot-linux-gnueabihf-gcc")
# set(JEVOIS_PLATFORM_CXX_COMPILER, "/usr/share/jevois-sdk/out/sun8iw5p1/linux/common/buildroot/host/usr/bin/arm-buildroot-linux-gnueabihf-g++")

# set(CMAKE_C_COMPILER "${JEVOIS_PLATFORM_C_COMPILER}")
# set(CMAKE_CXX_COMPILER "${JEVOIS_PLATFORM_CXX_COMPILER}")
set(CMAKE_C_COMPILER "/usr/share/jevois-sdk/out/sun8iw5p1/linux/common/buildroot/host/usr/bin/arm-buildroot-linux-gnueabihf-gcc")
set(CMAKE_CXX_COMPILER "/usr/share/jevois-sdk/out/sun8iw5p1/linux/common/buildroot/host/usr/bin/arm-buildroot-linux-gnueabihf-g++")

# set(CMAKE_C_COMPILER /usr/bin/arm-linux-gnueabihf-gcc)
# set(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-g++)

set(JEVOIS_PLATFORM_ARCH_FLAGS "-mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard -ftree-vectorize -Ofast -funsafe-math-optimizations -mfp16-format=ieee")

set(JEVOIS_PLATFORM_CFLAGS "-W -Wall -Wextra -g -O4 -I/usr/share/jevois-sdk/out/sun8iw5p1/linux/common/buildroot/build/linux-headers-3.4.113/usr/include -fPIC -Wno-deprecated-declarations -Wno-psabi") #-I/jevois/include 

set(CMAKE_C_FLAGS "-std=gnu99 ${JEVOIS_PLATFORM_ARCH_FLAGS} ${JEVOIS_PLATFORM_CFLAGS}")
set(CMAKE_CXX_FLAGS "${JEVOIS_PLATFORM_ARCH_FLAGS} ${JEVOIS_PLATFORM_CFLAGS}")

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set the project name and version
project(Conductor VERSION 0.1)

# add the sources subdirectory
INCLUDE_DIRECTORIES(Sources)
INCLUDE_DIRECTORIES(Headers)

# add include directories
# target_include_directories(Conductor PUBLIC "${PROJECT_BINARY_DIR}/Headers")

# include boost
# set(Boost_LIBRARIES, "usr/include/boost")
set(Boost_LIBRARIES, "/usr/share/jevois-sdk/out/sun8iw5p1/linux/common/buildroot/host/arm-buildroot-linux-gnueabihf/sysroot/usr/include/boost")
INCLUDE_DIRECTORIES(${Boost_LIBRARIES})

set(JSON_BuildTests OFF CACHE INTERNAL "")
set(JSON_Install OFF CACHE INTERNAL "")
add_subdirectory(json)

# add the executable
add_executable(Main Sources/main.cpp Sources/conductor.cpp)

# add_library(conductor_lib STATIC Sources/conductor.cpp)


# link boost library
TARGET_LINK_LIBRARIES( Main LINK_PUBLIC ${Boost_LIBRARIES} -lboost_system -lboost_regex -lboost_thread -lpthread -ltbb -ldl -lutil -lm)
TARGET_LINK_LIBRARIES( Main PRIVATE nlohmann_json::nlohmann_json)

# target_link_libraries(jevois ${JEVOIS_OPENCV_LIBS} -lpthread -ltbb -ldl -lutil -lboost_system -lboost_thread -lboost_regex -lturbojpeg -lm ${JEVOIS_PYTHON_LIBS})






