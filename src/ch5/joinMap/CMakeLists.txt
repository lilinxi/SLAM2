cmake_minimum_required(VERSION 3.14)
project(JoinMap)
set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)

FIND_PACKAGE(PCL 1.7 REQUIRED)
ADD_DEFINITIONS(${PCL_DEFINITIONS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_LIBRARIES(${PCL_LIBRARY_DIRS})

add_executable(joinMap joinMap.cpp)
target_link_libraries(joinMap ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${PCL_LIBRARIES})