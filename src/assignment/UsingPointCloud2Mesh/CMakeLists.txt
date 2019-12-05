CMAKE_MINIMUM_REQUIRED(VERSION 3.14) #设定版本
PROJECT(UsingPointCloud2Mesh) #设定工程名
set(CMAKE_CXX_STANDARD 14) #PCL requires C++14 or above

#设定可执行二进制文件的目录
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#增加子文件夹，也就是进入源代码文件夹继续构建
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src)
