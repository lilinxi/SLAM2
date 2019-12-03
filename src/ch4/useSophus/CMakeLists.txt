cmake_minimum_required(VERSION 3.14)
project(UseSophus)
set(CMAKE_CXX_STANDARD 14)

find_package(Sophus REQUIRED)
include_directories(${SOPHUS_INCLUDE_DIRS})
include_directories(/usr/local/include)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(useSophus useSophus.cpp)
