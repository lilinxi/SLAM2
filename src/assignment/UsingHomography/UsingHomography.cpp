/**
 * 实现虚拟广告牌的效果。
 * 提供两张图，一张是“计算机视觉life”公众号的logo，另外一张是带广告牌的原图，请用单应矩阵实现将原图中广告牌替换为提供的logo的效果。
 * 利用OpenCV函数，通过鼠标点击来选择要替换的广告牌的四个顶点。
 *
 * 本程序学习目标：
 * 理解掌握单应矩阵的使用
 */

#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>

using namespace cv;
using namespace std;

struct userdata {
    Mat im;
    vector<Point2f> points;
};

void mouseHandler(int event, int x, int y, int flags, void *data_ptr) {
    if (event == EVENT_LBUTTONDOWN) {
        userdata *data = ((userdata *) data_ptr);
        if (data->points.size() < 4) {
            circle(data->im, Point(x, y), 3, Scalar(0, 255, 255), 5, LINE_AA);
            imshow("Image", data->im);
            data->points.emplace_back(x, y);
            cout << "add: " << data->points.size() << endl;
        } else {
            data->points.clear();
            cout << "clear: " << data->points.size() << endl;
        }
    }
}

int main(int argc, char **argv) {
    // Read in the image.
    Mat im_src = imread("../cvlife.jpg");
    Size size = im_src.size();

    // Create a vector of points.
    vector<Point2f> pts_src;
    pts_src.emplace_back(0, 0);
    pts_src.emplace_back(size.width - 1, 0);
    pts_src.emplace_back(size.width - 1, size.height - 1);
    pts_src.emplace_back(0, size.height - 1);

    // Destination image
    Mat im_dst = imread("../ad.jpg");

    // Set data for mouse handler
    Mat im_temp = im_dst.clone();
    userdata data;
    data.im = im_temp;

    //show the image
    imshow("Image", im_temp);

    cout << "Click on four corners of a billboard and then press ENTER" << endl;
    //set the callback function for any mouse event
    setMouseCallback("Image", mouseHandler, &data);
    waitKey(0);

    // ----------  开始你的代码  --------------
    Mat H = findHomography(pts_src, data.points, 0);
    warpPerspective(im_src, data.im, H, data.im.size());
//    addWeighted(data.im, 0.5, im_dst, 0.5, 0.0, data.im);

    for (size_t y = 0; y < im_dst.rows; y++) {
        for (size_t x = 0; x < im_dst.cols; x++) {
            // 访问位于 x,y 处的像素
            // 用cv::Mat::ptr获得图像的行指针
            unsigned char *row_ptr = im_dst.ptr<unsigned char>(y);  // row_ptr是第y行的头指针
            unsigned char *data_ptr = &row_ptr[x * im_dst.channels()]; // data_ptr 指向待访问的像素数据

            unsigned char *row_ptr1 = data.im.ptr<unsigned char>(y);  // row_ptr是第y行的头指针
            unsigned char *data_ptr1 = &row_ptr1[x * data.im.channels()]; // data_ptr 指向待访问的像素数据
            // 输出该像素的每个通道,如果是灰度图就只有一个通道
            for (int c = 0; c != im_dst.channels(); c++) {
                // unsigned char data = data_ptr[c]; // data为I(x,y)第c个通道的值
                if (data_ptr1[c] > 0) {
                    data_ptr[c] = data_ptr1[c];
                }
            }
        }
    }
    // ----------  结束你的代码  --------------

    // Display image.
    imshow("Image", im_dst);
//    imshow("Image_data", data.im);
    waitKey(0);

    return 0;
}
