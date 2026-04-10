#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <Kinect.h>
#include <chrono>

using namespace std;
using namespace cv;


cv::Point2i selectedPoint(-1, -1);  // 存储选中的点坐标

void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {  // 鼠标左键按下事件
        selectedPoint = cv::Point2i(x, y);
        std::cout << "选中点坐标：" << selectedPoint << std::endl;
    }
}

int main() {
    for (int i = 0; i < 10;i++) {

        ostringstream oss;  //把数字转换为字符串
        oss << i;

        string outputPath = "D:/桌面/data/output";     // 指定保存图像的目录
        string filename = "_" + oss.str() + ".jpg";             // 指定保存图像的文件名

        string outputFullPath = outputPath + filename;
        cv::Mat image = cv::imread(outputFullPath);  // 读取图像


        cv::namedWindow("Image");
        cv::setMouseCallback("Image", onMouse);  // 设置鼠标事件回调函数

        while (true) {
            cv::imshow("Image", image);

            char key = cv::waitKey(1);
            if (key == 27 || key == 'q') {  // 按下ESC键或'q'键退出循环
                break;
            }
        }

        cv::destroyAllWindows();
    }
 

    return 0;
}