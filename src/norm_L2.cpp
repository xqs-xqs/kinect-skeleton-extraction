#include<iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

int main() {

	Point2i x;
	int x1, x2, y1,y2, s1, s2;
	Point2i y;
	Point2i stand;

	for (int i = 0; i < 10; i++) {
		cout << "输入人工标注点 + 改进算法的关节点 + kinect算法中的关节点 (先x后y)" << endl;
		cin >> s1 >> s2  >> x1 >> x2 >> y1 >>y2;
		stand = Point2i(s1, s2);
		x = Point2i(x1, x2);
		y = Point2i(y1, y2);
		
		float kinectI = norm(stand - y);
		float selfI = norm(stand - x);
		cout << "第" << i << "帧图像中kinect的误差为 = " << kinectI << endl;
		cout << "第" << i << "帧图像中改进算法的误差为 = " << selfI << endl<<endl;

	}
	


	return 0;
}