cmake_minimum_required(VERSION 3.0.0)
project(DroneCamera VERSION 0.1.0)

include(CTest)
enable_testing()


include_directories(src)
add_executable(DroneCamera src/main.cpp src/navigation.cpp src/fc_serial.cpp)

find_library(PIGPIOD_IF2 NAMES pigpiod_if2)
target_link_libraries(DroneCamera ${PIGPIOD_IF2})
find_package (Threads)
target_link_libraries (DroneCamera ${CMAKE_THREAD_LIBS_INIT})
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries( DroneCamera ${OpenCV_LIBS} )
add_definitions(-DASIO_STANDALONE)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
