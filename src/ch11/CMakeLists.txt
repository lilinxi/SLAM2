cmake_minimum_required(VERSION 3.14)
project(LoopClosure)
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

find_package(DBoW3 REQUIRED)
message(STATUS "DBoW3 library status:")
message(STATUS "    config: ${DBoW3_DIR}")
message(STATUS "    libraries: ${DBoW3_LIBS}")
message(STATUS "    include path: ${DBoW3_INCLUDE_DIRS}")

add_executable(feature_training feature_training.cpp)
target_link_libraries(feature_training ${OpenCV_LIBS} ${DBoW3_LIBS})

add_executable(loop_closure loop_closure.cpp)
target_link_libraries(loop_closure ${OpenCV_LIBS} ${DBoW3_LIBS})

add_executable(gen_vocab gen_vocab_large.cpp)
target_link_libraries(gen_vocab ${OpenCV_LIBS} ${DBoW3_LIBS})