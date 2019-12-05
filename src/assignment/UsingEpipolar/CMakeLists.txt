cmake_minimum_required(VERSION 3.14)
project(UsingEpipolar)
set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

add_executable(main UsingEpipolar.cpp)
target_link_libraries(main LINK_PRIVATE ${OpenCV_LIBS})



