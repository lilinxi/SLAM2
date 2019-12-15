#pragma once

#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>
#include "myslam/common_include.h"

namespace myslam {

    struct Frame;
    struct MapPoint;

/**
 * 2D 特征点
 * 在三角化之后会被关联一个地图点
 */
    struct Feature {
    public:
//        Eigen为了提高运算速度，采取了128位内存对齐，以让编译器进行向量化优化。
//        而如果自己的new就不会有内存对已，因此需要加上一个宏，重新实现内存对齐的new.
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr <Feature> Ptr;

//        避免相互引用
        std::weak_ptr <Frame> frame_;           // 持有该feature的frame
        cv::KeyPoint position_;                 // 2D提取位置
        std::weak_ptr <MapPoint> map_point_;    // 关联地图点

        bool is_outlier_ = false;       // 是否为异常点
        bool is_on_left_image_ = true;  // 标识是否提在左图，false为右图

    public:
        Feature() {}

        Feature(std::shared_ptr <Frame> frame, const cv::KeyPoint &kp)
                : frame_(frame), position_(kp) {}
    };
}  // namespace myslam

#endif  // MYSLAM_FEATURE_H
