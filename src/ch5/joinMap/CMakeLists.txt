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

#find_package(PCL REQUIRED COMPONENT common io)
#find_package(PCL REQUIRED COMPONENT common io visualization)
#include_directories(${PCL_INCLUDE_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#message(STATUS "    include path: ${PCL_INCLUDE_DIRS}")
#message(STATUS "    definitions: ${PCL_DEFINITIONS}")
#message(STATUS "    libraries: ${PCL_LIBRARIES}")
# 必须手动添加 visualization 库
#link_libraries(/usr/local/Cellar/pcl/1.9.1_4/lib/libpcl_visualization.dylib)

add_executable(joinMap joinMap.cpp)
target_link_libraries(joinMap ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
#target_link_libraries(joinMap ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${PCL_LIBRARIES})