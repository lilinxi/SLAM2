/**
 * 已知相机的位姿用四元数表示为 q=[0.39, 0.58, 0.19, 0.68]，顺序为 x,y,z,w，请编程实现：
 * 输出四元数对应的旋转矩阵、旋转矩阵的转置，旋转矩阵的逆矩阵，旋转矩阵乘以自身的转置，验证旋转矩阵的正交性。
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

using namespace std;
using namespace Eigen;

//• 旋转矩阵(3 × 3):        Eigen::Matrix3d。
//• 旋转向量(3 × 1):        Eigen::AngleAxisd。
//• 欧拉角(3 × 1):          Eigen::Vector3d。
//• 四元数(4 × 1):          Eigen::Quaterniond。
//• 欧氏变换矩阵(4 × 4):     Eigen::Isometry3d。
//• 仿射变换(4 × 4):        Eigen::Affine3d。
//• 射影变换(4 × 4):        Eigen::Projective3d。

int main(int argc, char **argv) {
    Eigen::Quaterniond quat(0.68, 0.39, 0.58, 0.19);
    Eigen::Matrix3d r = quat.toRotationMatrix();

    cout << "四元数 q =\n" << quat.coeffs() << endl;
    cout << "旋转矩阵 r =\n" << r << endl;
    cout << "旋转矩阵的转置 rt =\n" << r.transpose() << endl;
    cout << "旋转矩阵的逆 invr =\n" << r.inverse() << endl;
    cout << "旋转矩阵乘转置 r*rt =\n" << r * r.transpose() << endl;

    return 0;
}