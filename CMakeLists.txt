cmake_minimum_required(VERSION 3.12.4)

option(JEVOIS_PLATFORM "Build for jevois platform" OFF)

# Platform defines
set(JEVOIS_PLATFORM_C_COMPILER "/usr/share/jevois-sdk/out/sun8iw5p1/linux/common/buildroot/host/usr/bin/arm-buildroot-linux-gnueabihf-gcc")
set(JEVOIS_PLATFORM_CXX_COMPILER "/usr/share/jevois-sdk/out/sun8iw5p1/linux/common/buildroot/host/usr/bin/arm-buildroot-linux-gnueabihf-g++")
set(JEVOIS_PLATFORM_ARCH_FLAGS "-mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard -ftree-vectorize -Ofast -funsafe-math-optimizations -mfp16-format=ieee")
set(JEVOIS_PLATFORM_CFLAGS " -O2 -fopenmp -I/usr/share/jevois-sdk/out/sun8iw5p1/linux/common/buildroot/build/linux-headers-3.4.113/usr/include -fPIC -Wno-deprecated-declarations -Wno-psabi ")

# Host defines
set(HOST_C_COMPILER "/usr/bin/gcc")
set(HOST_CXX_COMPILER "/usr/bin/g++")
set(HOST_ARCH_FLAGS "")
set(HOST_CFLAGS " -fopenmp -O2 -fPIC -Wno-deprecated-declarations -Wno-psabi ")

set(CMAKE_SYSTEM_NAME Linux)
if(JEVOIS_PLATFORM)
    set(CMAKE_SYSTEM_PROCESSOR arm)
    set(CMAKE_C_COMPILER ${JEVOIS_PLATFORM_C_COMPILER})
    set(CMAKE_CXX_COMPILER ${JEVOIS_PLATFORM_CXX_COMPILER})
    set(CMAKE_C_FLAGS "-std=gnu99 -W -Wno-attribute-warning ${JEVOIS_PLATFORM_ARCH_FLAGS} ${JEVOIS_PLATFORM_CFLAGS}")
    set(CMAKE_CXX_FLAGS " -W -Wno-attribute-warning ${JEVOIS_PLATFORM_ARCH_FLAGS} ${JEVOIS_PLATFORM_CFLAGS}")
    set(BOOST_LIBRARIES, "/usr/share/jevois-sdk/out/sun8iw5p1/linux/common/buildroot/host/arm-buildroot-linux-gnueabihf/sysroot/usr/include/boost")
    message(STATUS "Compiling for Jevois platform")
else()
    set(CMAKE_C_COMPILER ${HOST_C_COMPILER})
    set(CMAKE_CXX_COMPILER ${HOST_CXX_COMPILER})
    set(CMAKE_C_FLAGS "-std=gnu99 -W -Wno-attribute-warning ${HOST_ARCH_FLAGS} ${HOST_CFLAGS}")
    set(CMAKE_CXX_FLAGS " -W -Wno-attribute-warning ${HOST_ARCH_FLAGS} ${HOST_CFLAGS}")
    set(BOOST_LIBRARIES, "usr/include/boost")
    message(STATUS "Compiling for host")
    add_compile_definitions(IS_HOST)
endif()

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set the project name and version
project(Conductor VERSION 0.1 LANGUAGES C CXX)

# add the sources subdirectory
INCLUDE_DIRECTORIES(Sources)
INCLUDE_DIRECTORIES(Headers)

# include boost
INCLUDE_DIRECTORIES(${BOOST_LIBRARIES})

set(JSON_BuildTests OFF CACHE INTERNAL "")
set(JSON_Install OFF CACHE INTERNAL "")
add_subdirectory(json)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/CRCpp/inc/)

# add the executables
set(SOURCES Sources/main.cpp Sources/conductor.cpp Sources/altitudeestimator.cpp Sources/altitudecontroller.cpp)
set(HEADERS Headers/conductor.h Headers/altitudeestimator.h Headers/altitudecontroller.h Headers/def.h Headers/my_enums.h CRCpp/inc/CRC.h)
add_executable(Main ${SOURCES} ${HEADERS})

# link libraries
TARGET_LINK_LIBRARIES( Main LINK_PUBLIC ${Boost_LIBRARIES} -lboost_system -lboost_regex -lboost_thread -lpthread -ltbb -ldl -lutil -lm)
TARGET_LINK_LIBRARIES( Main PRIVATE nlohmann_json::nlohmann_json)
