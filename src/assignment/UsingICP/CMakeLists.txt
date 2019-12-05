cmake_minimum_required(VERSION 2.8)
project(UsingICP)
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${SOPHUS_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

add_executable(UsingICP UsingICP.cpp)
target_link_libraries(UsingICP LINK_PRIVATE ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
