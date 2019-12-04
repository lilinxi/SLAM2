/**
 * 题目：点云融合实验。已经给定3帧（不连续）RGB-D相机拍摄的 RGB + depth 图像，以及他们之间的变换矩阵（以第一帧为参考帧），
 * 请将上述3帧RGB-D图像分别生成点云并融合出最终的点云输出。
 *
 * 本程序学习目标：
 * 熟悉PCL的使用。
 * 熟悉RGB-D图像到点云的转换过程。
 * 掌握简单的点云融合方法。
 */

#include "slamBase.hpp"
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv) {
    CAMERA_INTRINSIC_PARAMETERS cameraParams{517.0, 516.0, 318.6, 255.3, 5000.0};
    int frameNum = 3;
    vector <Eigen::Isometry3d> poses;
    PointCloud::Ptr fusedCloud(new PointCloud());
    string path = "../data/";
    string cameraPosePath = path + "cameraTrajectory.txt";
    readCameraTrajectory(cameraPosePath, poses);
    for (int idx = 0; idx < frameNum; idx++) {
        string rgbPath = path + "rgb/rgb" + to_string(idx) + ".png";
        string depthPath = path + "depth/depth" + to_string(idx) + ".png";

        FRAME frm;
        frm.rgb = cv::imread(rgbPath);
        if (frm.rgb.empty()) {
            cerr << "Fail to load rgb image!" << endl;
        }
        frm.depth = cv::imread(depthPath, -1);
        if (frm.depth.empty()) {
            cerr << "Fail to load depth image!" << endl;
        }

        if (idx == 0) {
            fusedCloud = image2PointCloud(frm.rgb, frm.depth, cameraParams);
        } else {
            fusedCloud = pointCloudFusion(fusedCloud, frm, poses[idx], cameraParams);
        }
    }
    pcl::io::savePCDFile("./fusedCloud.pcd", *fusedCloud);

    // 方便起见这里使用指针的形式
    PointCloud::Ptr cloud(new PointCloud);
    if (pcl::io::loadPCDFile("./fusedCloud.pcd", *cloud) == -1) {
        PCL_ERROR("can not read file");
        return 0;
    }

    // 展示 map.pcd 中的内容
    pcl::visualization::PCLVisualizer viewer;
    viewer.addPointCloud(cloud, "cloud");
    viewer.spin();
    return 0;
}

