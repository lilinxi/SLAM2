cmake_minimum_required(VERSION 3.14)
project(UsingEigen)
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(main UsingEigen.cpp)
