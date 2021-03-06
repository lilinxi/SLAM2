cmake_minimum_required(VERSION 3.14)
project(2D2D)
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

find_package(G2O REQUIRED)
find_package(Sophus REQUIRED)
include_directories(
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
)

add_executable(2d2d pose_estimation_2d2d.cpp)
target_link_libraries(2d2d ${OpenCV_LIBS})
