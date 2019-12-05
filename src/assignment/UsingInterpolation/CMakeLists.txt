cmake_minimum_required(VERSION 3.14)
project(UsingInterpolation)
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

add_executable(UsingInterpolation UsingInterpolation.cpp)
target_link_libraries(UsingInterpolation ${OpenCV_LIBS})