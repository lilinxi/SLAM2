/**
 * 请使用 C++ 新特性改写以下函数。该函数功能：将一组无序的坐标按照“Z”字形排序，并输出。
 * 熟悉 C++ 新特性（简化循环、自动类型推导、列表初始化、lambda函数）
 */

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

bool cmp(const Point2i &pt1, const Point2i &pt2) {
    return pt1.x == pt2.x ? pt1.y < pt2.y : pt1.x < pt2.x;
}

int main(int argc, char **argv) {
    // 列表初始化
    vector <Point2i> vec{Point2i(2, 1), Point2i(3, 3)};
    vec.emplace_back(2, 3);
    vec.emplace_back(3, 2);
    vec.emplace_back(3, 1);
    vec.emplace_back(1, 3);
    vec.emplace_back(1, 1);
    vec.emplace_back(2, 2);
    vec.emplace_back(1, 2);

    cout << "Before sort: " << endl;
    // 自动类型推导，简化循环
    for (const auto &v:vec) {
        cout << v << endl;
    }

    sort(vec.begin(), vec.end(), cmp);

    cout << "After sort: " << endl;
    // lambda 表达式
    for_each(vec.begin(), vec.end(), [](const Point2i &p) { cout << p << endl; });

    return 0;
}




