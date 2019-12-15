cmake_minimum_required(VERSION 3.14)
project(MySLAM)
set(CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

############### dependencies ######################
# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
message(STATUS "EIGEN3 library status:")
message(STATUS "    config: ${EIGEN3_DIR}")
message(STATUS "    version: ${EIGEN3_VERSION}")
message(STATUS "    include path: ${EIGEN3_INCLUDE_DIRS}")

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
message(STATUS "Pangolin library status:")
message(STATUS "    config: ${Pangolin_DIR}")
message(STATUS "    version: ${Pangolin_VERSION}")
message(STATUS "    libraries: ${Pangolin_LIBRARIES}")
message(STATUS "    include path: ${Pangolin_INCLUDE_DIRS}")

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
message(STATUS "Sophus library status:")
message(STATUS "    config: ${Sophus_DIR}")
message(STATUS "    version: ${Sophus_VERSION}")
message(STATUS "    libraries: ${Sophus_LIBS}")
message(STATUS "    include path: ${Sophus_INCLUDE_DIRS}")

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
message(STATUS "G2O library status:")
message(STATUS "    config: ${G2O_DIR}")
message(STATUS "    version: ${G2O_VERSION}")
message(STATUS "    libraries: ${G2O_LIBS}")
message(STATUS "    include path: ${G2O_INCLUDE_DIRS}")

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})
message(STATUS "GLOG library status:")
message(STATUS "    config: ${GLOG_DIR}")
message(STATUS "    version: ${GLOG_VERSION}")
message(STATUS "    libraries: ${GLOG_LIBS}")
message(STATUS "    include path: ${GLOG_INCLUDE_DIRS}")

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
message(STATUS "GTEST library status:")
message(STATUS "    config: ${GTEST_DIR}")
message(STATUS "    version: ${GTEST_VERSION}")
message(STATUS "    libraries: ${GTEST_LIBS}")
message(STATUS "    libraries: ${GTEST_BOTH_LIBRARIES}")
message(STATUS "    include path: ${GTEST_INCLUDE_DIRS}")

# gflags
#find_package(GFlags REQUIRED)
#include_directories(${GFLAGS_INCLUDE_DIRS})
#message(STATUS "GFLAGS library status:")
#message(STATUS "    config: ${GFLAGS_DIR}")
#message(STATUS "    version: ${GFLAGS_VERSION}")
#message(STATUS "    libraries: ${GFLAGS_LIBS}")
#message(STATUS "    include path: ${GFLAGS_INCLUDE_DIRS}")

# csparse
#find_package(CSparse REQUIRED)
#include_directories(${CSPARSE_INCLUDE_DIR})
#message(STATUS "CSPARSE library status:")
#message(STATUS "    config: ${CSPARSE_DIR}")
#message(STATUS "    version: ${CSPARSE_VERSION}")
#message(STATUS "    libraries: ${CSPARSE_LIBS}")
#message(STATUS "    include path: ${CSPARSE_INCLUDE_DIRS}")

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${GLOG_LIBRARIES}
        ${GTEST_BOTH_LIBRARIES}
        #        ${GFLAGS_LIBRARIES}
        pthread
        #        ${CSPARSE_LIBRARY}
        )

enable_testing()

############### source and test ######################
include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${PROJECT_BINARY_DIR}/lib)
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(app)

#TODO: 兜底
include_directories(/usr/local/include)
link_directories(/usr/local/lib)