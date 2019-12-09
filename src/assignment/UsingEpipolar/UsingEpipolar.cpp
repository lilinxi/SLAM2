/**
 * 题目：现有一个运动着的相机拍摄的连续两张图片，其中特征点匹配部分已经完成。
 * 请根据两帧图像对应的匹配点计算基础矩阵，并利用该矩阵绘制出前10个特征点对应的极线。
 *
 * 本程序学习目标：
 * 理解掌握对极约束的原理
 * 熟悉OpenCV编程
 */

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    Mat rgb1 = imread("./rgb1.ppm");
    Mat rgb2 = imread("./rgb2.ppm");

//    Ptr<FeatureDetector> detector;
    Ptr<ORB> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor;

    // TODO bug : https://www.cnblogs.com/wangguchangqing/p/8076061.html
//    detector = cv::DescriptorMatcher::create("ORB");
    descriptor = cv::DescriptorMatcher::create("ORB");

    vector<KeyPoint> kp1, kp2;
    detector->detect(rgb1, kp1);
    detector->detect(rgb2, kp2);

    // 计算描述子
    Mat desp1, desp2;
//    descriptor->compute(rgb1, kp1, desp1);
//    descriptor->compute(rgb2, kp2, desp2);
    detector->compute(rgb1, kp1, desp1);
    detector->compute(rgb2, kp2, desp2);

    // 匹配描述子
    vector<DMatch> matches;
    BFMatcher matcher;
    matcher.match(desp1, desp2, matches);
    cout << "Find total " << matches.size() << " matches." << endl;

    // 筛选匹配对
    vector<DMatch> goodMatches;
    double minDis = 9999;
    for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i].distance < minDis) {
            minDis = matches[i].distance;
        }
    }

    for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i].distance < 10 * minDis) {
            goodMatches.push_back(matches[i]);
        }
    }

    vector<Point2f> pts1, pts2;
    for (size_t i = 0; i < goodMatches.size(); i++) {
        pts1.push_back(kp1[goodMatches[i].queryIdx].pt);
        pts2.push_back(kp2[goodMatches[i].trainIdx].pt);
    }

    // 请先计算基础矩阵并据此绘制出前10个匹配点对应的对极线，可以调用opencv函数
    // ----------- 开始你的代码 --------------//

    Mat F;
    F = findFundamentalMat(pts1, pts2, FM_RANSAC);
    cout << "基础矩阵F:" << endl << F << endl;

    vector<Vec3f> linesl;
    computeCorrespondEpilines(pts1, 1, F, linesl);

    for (auto it = linesl.begin(); it != linesl.end(); it++) {
        line(rgb1, Point(0, -(*it)[2] / (*it)[1]), Point(rgb1.cols, -((*it)[2] + (*it)[0] * rgb1.cols) / (*it)[1]),
             Scalar(255, 255, 255));
    }

    vector<Vec3f> lines2;
    computeCorrespondEpilines(pts2, 2, F, lines2);
    for (auto it = lines2.begin(); it != lines2.end(); it++) {
        line(rgb2, Point(0, -(*it)[2] / (*it)[1]), Point(rgb2.cols, -((*it)[2] + (*it)[0] * rgb2.cols) / (*it)[1]),
             Scalar(255, 255, 255));
    }

    // ----------- 结束你的代码 --------------//

    imshow("epiline1", rgb2);
    imshow("epiline2", rgb1);
    waitKey(0);

    return 0;
}
