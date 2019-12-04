#include "slamBase.hpp"

PointCloud::Ptr image2PointCloud(Mat rgb, Mat depth, CAMERA_INTRINSIC_PARAMETERS camera) {
    PointCloud::Ptr cloud(new PointCloud);
    cout << "image to point cloud" << endl;
    for (int m = 0; m < depth.rows; m++) {
        for (int n = 0; n < depth.cols; n++) {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;
            // 计算这个点的空间坐标
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            // 从rgb图像中获取它的颜色
            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

            // 把p加入到点云中
            cloud->points.push_back(p);
        }
    }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    return cloud;
}

PointCloud::Ptr
pointCloudFusion(PointCloud::Ptr &original, FRAME &newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS camera) {
    // ---------- 开始你的代码  ------------- -//
    Mat rgb = newFrame.rgb;
    Mat depth = newFrame.depth;

    cout << "merge image to point cloud" << endl;
    for (int m = 0; m < depth.rows; m++) {
        for (int n = 0; n < depth.cols; n++) {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;

            // d 存在值，则向点云增加一个点
            PointT p;
            // 计算这个点的空间坐标
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            Eigen::Vector3d v(p.x, p.y, p.z);
            v = T * v;
            p.x = v[0];
            p.y = v[1];
            p.z = v[2];

            // 从rgb图像中获取它的颜色
            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

            // 把p加入到点云中
            original->points.push_back(p);
        }
    }
    // 设置并保存点云
    original->height = 1;
    original->width = original->points.size();
    original->is_dense = false;
    return original;
    // ---------- 结束你的代码  ------------- -//
}

// tx ty tz qx qy qz qw
void readCameraTrajectory(string camTransFile, vector<Eigen::Isometry3d> &poses) {
    // ---------- 开始你的代码  ------------- -//
    ifstream fin(camTransFile);
    if (!fin) {
        cout << "cannot find trajectory file at " << camTransFile << endl;
    }

    while (!fin.eof()) {
        double tx, ty, tz, qx, qy, qz, qw;
        fin >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));
        Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
        poses.push_back(Twr);
    }
    cout << "read total " << poses.size() << " pose entries" << endl;
    // ---------- 结束你的代码  ------------- -//
}