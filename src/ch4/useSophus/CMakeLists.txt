cmake_minimum_required(VERSION 3.14)
project(UseSophus)
set(CMAKE_CXX_STANDARD 14)

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(useSophus useSophus.cpp)
