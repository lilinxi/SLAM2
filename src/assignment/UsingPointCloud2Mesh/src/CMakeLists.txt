
FIND_PACKAGE(PCL REQUIRED)

FIND_PACKAGE(OpenCV REQUIRED)

ADD_DEFINITIONS(${PCL_DEFINITIONS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_LIBRARIES(${PCL_LIBRARY_DIRS})

ADD_EXECUTABLE(pointCloud2Mesh pointCloud2Mesh.cpp)
TARGET_LINK_LIBRARIES(pointCloud2Mesh
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        /usr/local/lib/libflann.dylib
        /usr/local/Cellar/flann/1.9.1_8/lib/libflann.dylib
        /usr/local/Cellar/flann/1.9.1_8/lib/libflann_cpp.dylib
        )




