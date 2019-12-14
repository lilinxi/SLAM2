cmake_minimum_required(VERSION 3.14)
project(3D2D)
set(CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

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
link_libraries(${G2O_LIBRARIES})
#TODO G2O and Sophus can not found
message(STATUS "    include path: ${G2O_INCLUDE_DIRS}")
message(STATUS "    libraries: ${G2O_LIBRARIES}")
message(STATUS "    libraries: ${G2O_LIBS}")
message(STATUS "    include path: ${SOPHUS_INCLUDE_DIRS}")
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(3d2d pose_estimation_3d2d.cpp)
target_link_libraries(3d2d
        g2o_core g2o_stuff
        ${OpenCV_LIBS})
