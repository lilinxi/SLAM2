
FIND_PACKAGE(OpenCV REQUIRED)

FIND_PACKAGE(PCL 1.7 REQUIRED)

ADD_DEFINITIONS(${PCL_DEFINITIONS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_LIBRARIES(${PCL_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(${PROJECT_SOURSE_DIR}/include)

ADD_LIBRARY(slambase_shared SHARED slamBase.cpp)
TARGET_LINK_LIBRARIES(slambase_shared
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES})


ADD_EXECUTABLE(pointCloudFusion pointCloudFusion.cpp)
TARGET_LINK_LIBRARIES(pointCloudFusion
        slambase_shared
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        )




