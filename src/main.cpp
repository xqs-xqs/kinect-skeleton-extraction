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
using namespace cv::ximgproc;
using namespace chrono;




//后续可能要整摄像机坐标系 即RGB空间下的骨架绘制 和前面一开始截的图做对比
//将摄像机坐标转换为深度空间坐标系下的坐标 并将两个关节点坐标连线  在深度空间下绘制骨架
void DrawLine(Mat& Img, const Joint& cameraJoint1, const Joint& cameraJoint2, ICoordinateMapper* pCoordinateMapper)
{
	if (cameraJoint1.TrackingState == TrackingState_NotTracked || cameraJoint2.TrackingState == TrackingState_NotTracked)  //关节状态正确
		return;

	DepthSpacePoint depthPoint1, depthPoint2;
	pCoordinateMapper->MapCameraPointToDepthSpace(cameraJoint1.Position, &depthPoint1);  //将摄像机坐标系转换为深度空间坐标系
	pCoordinateMapper->MapCameraPointToDepthSpace(cameraJoint2.Position, &depthPoint2);

	line(Img, Point(depthPoint1.X, depthPoint1.Y), Point(depthPoint2.X, depthPoint2.Y), Scalar(0, 0, 255), 5); //在单通道中代表白色 三通道中代表红色
}


//得到除手臂以外骨架关节点  手臂不画 共15个关节点
void GetBodyJunctions(Mat& Image, IBody* pBody, Joint aJoints[JointType_Count], ICoordinateMapper* pCoordinateMapper) {

	DrawLine(Image, aJoints[JointType_SpineBase], aJoints[JointType_SpineMid], pCoordinateMapper);
	DrawLine(Image, aJoints[JointType_SpineMid], aJoints[JointType_SpineShoulder], pCoordinateMapper);
	DrawLine(Image, aJoints[JointType_SpineShoulder], aJoints[JointType_Neck], pCoordinateMapper);
	DrawLine(Image, aJoints[JointType_Neck], aJoints[JointType_Head], pCoordinateMapper);

	DrawLine(Image, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderLeft], pCoordinateMapper);
	DrawLine(Image, aJoints[JointType_ShoulderLeft], aJoints[JointType_ElbowLeft], pCoordinateMapper);
	//DrawLine(Image, aJoints[JointType_ElbowLeft], aJoints[JointType_WristLeft], pCoordinateMapper);
	//DrawLine(Image, aJoints[JointType_WristLeft], aJoints[JointType_HandLeft], pCoordinateMapper);
	//DrawLine(Image, aJoints[JointType_HandLeft], aJoints[JointType_HandTipLeft], pCoordinateMapper);
	//DrawLine(Image, aJoints[JointType_HandLeft], aJoints[JointType_ThumbLeft], pCoordinateMapper);

	DrawLine(Image, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderRight], pCoordinateMapper);
	DrawLine(Image, aJoints[JointType_ShoulderRight], aJoints[JointType_ElbowRight], pCoordinateMapper);
	//DrawLine(Image, aJoints[JointType_ElbowRight], aJoints[JointType_WristRight], pCoordinateMapper);
	//DrawLine(Image, aJoints[JointType_WristRight], aJoints[JointType_HandRight], pCoordinateMapper);
	//DrawLine(Image, aJoints[JointType_HandRight], aJoints[JointType_HandTipRight], pCoordinateMapper);
	//DrawLine(Image, aJoints[JointType_HandRight], aJoints[JointType_ThumbRight], pCoordinateMapper);

	DrawLine(Image, aJoints[JointType_SpineBase], aJoints[JointType_HipLeft], pCoordinateMapper);
	DrawLine(Image, aJoints[JointType_HipLeft], aJoints[JointType_KneeLeft], pCoordinateMapper);
	DrawLine(Image, aJoints[JointType_KneeLeft], aJoints[JointType_AnkleLeft], pCoordinateMapper);
	DrawLine(Image, aJoints[JointType_AnkleLeft], aJoints[JointType_FootLeft], pCoordinateMapper);

	DrawLine(Image, aJoints[JointType_SpineBase], aJoints[JointType_HipRight], pCoordinateMapper);
	DrawLine(Image, aJoints[JointType_HipRight], aJoints[JointType_KneeRight], pCoordinateMapper);
	DrawLine(Image, aJoints[JointType_KneeRight], aJoints[JointType_AnkleRight], pCoordinateMapper);
	DrawLine(Image, aJoints[JointType_AnkleRight], aJoints[JointType_FootRight], pCoordinateMapper);
}


int main()
{
	//得到Kinect传感器指针
	cout << "Try to get default sensor" << endl;
	IKinectSensor* pSensor = nullptr;
	if (GetDefaultKinectSensor(&pSensor) != S_OK)  //获取指针的同时，进行状态检查
	{
		cerr << "Get Sensor failed" << endl;
		return -1;
	}

	// 打开Kinect传感器
	cout << "Try to open sensor" << endl;
	if (pSensor->Open() != S_OK)
	{
		cerr << "Can't open sensor" << endl;
		return -1;
	}

	//通过传感器获取depth的Srouce
	cout << "Try to get  depth source" << endl;
	IDepthFrameSource* pDepthSource = nullptr;
	if (pSensor->get_DepthFrameSource(&pDepthSource) != S_OK)
	{
		cerr << "Can't get depth frame source" << endl;
		return -1;
	}

	//通过传感器获取Color的Srouce
	cout << "Try to get  color source" << endl;
	IColorFrameSource* pColorSource = nullptr;
	if (pSensor->get_ColorFrameSource(&pColorSource) != S_OK)
	{
		cerr << "Can't get color frame source" << endl;
		return -1;
	}


	//通过传感器获取Body的Srouce
	cout << "Try to get  body source" << endl;
	IBodyFrameSource* pBodySource = nullptr;
	if (pSensor->get_BodyFrameSource(&pBodySource) != S_OK)
	{
		cerr << "Can't get body frame source" << endl;
		return -1;
	}

	//通过传感器获取BodyIndex的Srouce
	cout << "Try to get bodyindex source" << endl;
	IBodyIndexFrameSource* pBodyIndexSource = nullptr;
	if (pSensor->get_BodyIndexFrameSource(&pBodyIndexSource) != S_OK)
	{
		cerr << "Can't get bodyindex frame source" << endl;
		return -1;
	}


	//获取Depth的Reader
	cout << "Try to get  depth reader" << endl;
	IDepthFrameReader* pDepthReader = nullptr;
	if (pDepthSource->OpenReader(&pDepthReader) != S_OK)
	{
		cerr << "Can't get depth frame reader" << endl;
		return -1;
	}

	//获取Color的Reader
	cout << "Try to get  color reader" << endl;
	IColorFrameReader* pColorReader = nullptr;
	if (pColorSource->OpenReader(&pColorReader) != S_OK)
	{
		cerr << "Can't get color frame reader" << endl;
		return -1;
	}

	//获取Body的Reader
	cout << "Try to get  body reader" << endl;
	IBodyFrameReader* pBodyReader = nullptr;
	if (pBodySource->OpenReader(&pBodyReader) != S_OK)
	{
		cerr << "Can't get body frame reader" << endl;
		return -1;
	}

	//获取BodyIndex的Reader
	cout << "Try to get  index reader" << endl;
	IBodyIndexFrameReader* pBodyIndexReader = nullptr;
	if (pBodyIndexSource->OpenReader(&pBodyIndexReader) != S_OK)
	{
		cerr << "Can't get index frame reader" << endl;
		return -1;
	}

	//获取深度图像相关数据 利用IFrameDescription接口
	cout << "Try to get depth description" << endl;
	cout << "Try to get bodyindex description" << endl;
	cout << "Try to get color description" << endl;
	int depthWidth = 0;
	int depthHeight = 0;
	int bodyIndexWidth = 0;
	int bodyIndexHeight = 0;
	int colorWidth = 0;
	int colorHeight = 0;
	USHORT depthMin = 0;
	USHORT depthMax = 0;

	IFrameDescription* pDepthDescription = nullptr;
	IFrameDescription* pBodyIndexDescription = nullptr;
	IFrameDescription* pColorDescription = nullptr;

	//深度图像高度宽度获取  424*512
	if (pDepthSource->get_FrameDescription(&pDepthDescription) == S_OK)
	{
		pDepthDescription->get_Width(&depthWidth);
		pDepthDescription->get_Height(&depthHeight);
	}
	//用户图像高宽获取
	if (pBodyIndexSource->get_FrameDescription(&pBodyIndexDescription) == S_OK) {
		pBodyIndexDescription->get_Width(&bodyIndexWidth);
		pBodyIndexDescription->get_Height(&bodyIndexHeight);
	}
	//彩色图像高宽获取
	if (pColorSource->get_FrameDescription(&pColorDescription) == S_OK) {
		pColorDescription->get_Width(&colorWidth);
		pColorDescription->get_Height(&colorHeight);
	}


	pDepthSource->get_DepthMinReliableDistance(&depthMin);   //获取深度距离最大 最小可靠深度  
	pDepthSource->get_DepthMaxReliableDistance(&depthMax);

	pDepthDescription->Release();
	pDepthDescription = nullptr;

	pBodyIndexDescription->Release();
	pBodyIndexDescription = nullptr;
	
	pColorDescription->Release();
	pColorDescription = nullptr;

	//创建坐标转换器
	cout << "Try to get coordinate mapper" << endl;
	ICoordinateMapper* pCoordinateMapper = nullptr;
	if (pSensor->get_CoordinateMapper(&pCoordinateMapper) != S_OK)
	{
		cout << "Can't get coordinate mapper" << endl;
		return -1;
	}


	//用于测试的定时模块
	int timer = 0; //时间计时器 当达到1000毫秒即1秒时就会记录数据 且清0
	int frameCount = 0; //表示当前以及采集了多少帧图像了


	cout << "enter main loop" << endl;
	//主体
	while (1) {

		//1. 获取深度图像和用户索引图像

		// 获取当前帧的深度图像和用户索引中真正的数据
		IDepthFrame* pDepthFrame = nullptr;
		IBodyIndexFrame* pBodyIndexFrame = nullptr;
		IBodyFrame* pBodyFrame = nullptr;
		IColorFrame* pColorFrame = nullptr;

		if (pDepthReader->AcquireLatestFrame(&pDepthFrame) == S_OK) {
			/*cout << "get depthFrame successfully" << endl;*/

			if (pBodyIndexReader->AcquireLatestFrame(&pBodyIndexFrame) == S_OK) {
				/*cout << "get bodyIndexFrame successfully" << endl;*/
				// 初始化Depth和Body数据 使其每次循环重新更新
				UINT16* pDepthData = nullptr;
				UINT depthDataSize = 0;
				BYTE* pBodyIndexData = nullptr;
				UINT bodyIndexDataSize = 0;



				//获取DepthFrame数据 使用SUCCEEDED宏进行判断 简洁代码
				HRESULT hr = pDepthFrame->AccessUnderlyingBuffer(&depthDataSize, &pDepthData);
				if (SUCCEEDED(hr))
				{
					/*cout << "get depthData successfully" << endl;*/
					hr = pBodyIndexFrame->AccessUnderlyingBuffer(&bodyIndexDataSize, &pBodyIndexData);
					
				}


				//2. 深度图像中的人体区域提取
				if (SUCCEEDED(hr)) {
					/*cout << "get bodyIndexData successfully" << endl;*/
					//创建深图像和用户索引图像的Mat对象
					Mat depthImage(depthHeight, depthWidth, CV_16U, pDepthData);
					Mat bodyIndexImage(bodyIndexHeight, bodyIndexWidth, CV_8U, pBodyIndexData);
					/*imshow("Depth Image", depthImage);
					imshow("BodyIndex Image", bodyIndexImage);*/


					//测试模块 深度图像中的人体提取
					auto bodyExtractionStart = steady_clock::now();  //记录当前开始的时间戳

				
					int userIndex = 0; //获取用户索引
					bool flag = FALSE;   //标记
					//在遍历用户索引图像的基础上 对深度图像中相应位置非用户索引的点就就设置为0xFFFF
					Mat bodyDepthImage(depthHeight, depthWidth, CV_16U);
					for (int y = 0; y < bodyIndexHeight; y++) {
						for (int x = 0; x < bodyIndexWidth; x++) {
							int index = y * bodyIndexWidth + x;
							if (pBodyIndexData[index] < 6 && (depthMin <= pDepthData[index] <= depthMax)) {
								bodyDepthImage.at<UINT16>(y, x) = pDepthData[index];
								if (!flag) {
									userIndex = pBodyIndexData[index]; //一次性获取用户索引值
									flag = TRUE;
								}
							}
							else
							{
								bodyDepthImage.at<UINT16>(y, x) = 0xFFFF;
							}
						}
					}
					imshow("BodyDepthImage", bodyDepthImage); //只包含人体的深度图像



					//对深度图像进行二值化 二值化的输出图像类型会和输入图像类型一样
					Mat bodyDepthBinaryImage(depthHeight, depthWidth, CV_16U);
					threshold(bodyDepthImage, bodyDepthBinaryImage, 0xfff0, 0xffff, THRESH_BINARY_INV);


					//图像预处理，去除噪声填补空洞
					//中值滤波
					medianBlur(bodyDepthBinaryImage, bodyDepthBinaryImage, 5);
					//数学形态学操作 多次开运算和闭运算操作
					Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));  //后面已经用过elment这个名了
					for (int i = 0; i < 2; i++) {
						morphologyEx(bodyDepthBinaryImage, bodyDepthBinaryImage, MORPH_OPEN, element);
						morphologyEx(bodyDepthBinaryImage, bodyDepthBinaryImage, MORPH_CLOSE, element); //是不是单独试一下 会更好
					}
					imshow("BodyDepthBinaryImage_Pre", bodyDepthBinaryImage);  //图像预处理结果


					auto bodyExtractionEnd = steady_clock::now();  //记录人体图像提取结束的时间戳
					if (timer == 500 && frameCount <= 10)  //当间隔1秒时 且采集的图像帧不超过10帧
					{
						auto bodyExtractionDuration = duration_cast<std::chrono::milliseconds>(bodyExtractionEnd - bodyExtractionStart);  //计算持续时间 并且将时间格式改为毫秒
						cout << "第"<<frameCount << "帧人体图像提取耗时: " << bodyExtractionDuration.count() << " 毫秒" << endl;
					}





					//测试模块 人体中段的手臂二值图提取
					auto armExtractionStart = steady_clock::now();  //记录当前开始的时间戳

					//得到手臂单独的二值图

					//摄像头离地高1.15米 人需要离2米外才能够将全身照进去，最好人在里摄像头2.35m-2.5m内比较好
					int topLineY = 130;  // 上方线的 Y 坐标
					int bottomLineY = 260;  // 下方线的 Y 坐标

					// 截取躯干部分的区域 为了精度更高 使用原本16位的深度图像
					Rect torsoRect(0, topLineY, bodyDepthImage.cols, bottomLineY - topLineY);
					//Mat torsoImage= Mat::ones(depthHeight, depthWidth, CV_16UC1) * 0xffff ; //显示为全白 最远距离0xffff 距离越远越白
					Mat torsoImage(depthHeight, depthWidth, CV_16UC1);
					bodyDepthImage(torsoRect).copyTo(torsoImage(torsoRect));

					// 显示截取后的躯干图像
					imshow("Torso Image", torsoImage);


					double sum_depth = 0.0;  // 深度值累加和
					int count = 0;			 // 符合条件的像素数量

					UINT16 min_depth = 1900;  // 指定的深度范围下限 相当于1.8m 物理约束
					UINT16 max_depth = 2350;  // 指定的深度范围上限 相当于2.5m

					//遍历截取出的躯干图像中深度值平均值
					for (int y = 130; y <= 260; y++) {
						for (int x = 0; x < depthWidth; x++) {
							double depth_value = static_cast<double>(torsoImage.at<UINT16>(y, x));  // 获取当前像素的深度值
							/*if(depth_value!= 65535  && depth_value >3200)
								cout << "UINT16深度值大小" << depth_value << endl;*/

							if (depth_value >= min_depth && depth_value <= max_depth) {
								sum_depth += depth_value;  // 累加深度值							
								count++;  // 记录符合条件的像素数量
							}

						}
					}

					double average_depth = (count > 0) ? (sum_depth / count) : 0.0; // 计算平均深度值
					/*cout << "深度平均值"<< average_depth << endl;
					cout << "有效像素点" << count << endl;*/

					// 提取深度值与平均值相差 40 以上的像素点到另一张图像中
					Mat diff_image(bodyDepthImage.size(), CV_8UC1, Scalar(0));  // 创建一张单通道图像，用于保存差异像素点
					for (int y = 130; y <= 260; y++) {
						for (int x = 0; x < depthWidth; x++) {
							double depth_value = static_cast<double>(torsoImage.at<UINT16>(y, x));

							if (depth_value >= min_depth && depth_value <= max_depth) {
								double diff = average_depth - depth_value;
								if (diff > 40) {
									diff_image.at<uint8_t>(y, x) = 255;  // 将符合条件的像素点设为白色
								}
							}
						}
					}

					//中值滤波
					medianBlur(diff_image, diff_image, 5);
					//数学形态学操作 多次开运算和闭运算操作
					/*Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));*/
					for (int i = 0; i < 5; i++)
						morphologyEx(diff_image, diff_image, MORPH_CLOSE, element);
					for (int i = 0; i < 5; i++)
						morphologyEx(diff_image, diff_image, MORPH_OPEN, element);

					imshow("DepthDiffImage_Arm", diff_image); //得到预处理过的遮挡手臂的二值图




					//测试模块 人体中段的手臂二值图提取
					auto armExtractionEnd = steady_clock::now();  //记录当前开始的时间戳
					if (timer == 500 && frameCount <= 10)  //当间隔1秒时 且采集的图像帧不超过10帧
					{
						auto armExtractionDuration = duration_cast<std::chrono::milliseconds>(armExtractionEnd - armExtractionStart);  //计算持续时间 并且将时间格式改为毫秒
						cout << "第"<<frameCount << "帧手臂提取提取耗时: " << armExtractionDuration.count() << " 毫秒" << endl;
					}




					//测试模块 人体骨架提取
					auto skeletonExtractionStart = steady_clock::now();  //记录当前开始的时间戳

					//判断手臂自遮挡情况判断

					Point2f elbowRight;  // 右肘
					Point2f elbowLeft;   // 左肘
					Point2f handRight;	 // 右手
					Point2f handLeft;	 // 左手
					Point2f handTipRight;	 // 左手
					Point2f handTipLeft;	 // 左手
					

					//躯干四边形范围
					float spineShoulderY = 0; //肩膀处的脊椎 Y1
					float spineBaseY = 0;     //脊椎底部  Y2
					float shoulderLeftX = 0;   //左肩 X1
					float shoulderRightX = 0;  //右肩 X2

					bool isLeftArmSelfOccluded = false;  //左手臂自遮挡
					bool isRightArmSelfOccluded = false; //右手臂自遮挡
					bool isArmSelfOccluded = false;  //手臂自遮挡判断

					//获取双手的手肘和手腕关节点位置
					IBody* pBody[BODY_COUNT] = { 0 };
					Joint joints[JointType_Count]; //25个关节点的数据
					hr = pBodyReader->AcquireLatestFrame(&pBodyFrame);
					if (SUCCEEDED(hr)) {
						hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
						if (SUCCEEDED(hr))
						{
							//for (int i = 0; i < BODY_COUNT; i++)  //检测多个人方案

							BOOLEAN isTracked = false;
							if (pBody[userIndex]->get_IsTracked(&isTracked) == S_OK)  //关节点追踪状态检测 （需嵌套使用）
							{
								/*Joint joints[JointType_Count];*/
								if (pBody[userIndex]->GetJoints(JointType_Count, joints) == S_OK)  //得到该用户自己的关节点数据
								{
									//坐标系转换 方便后续进行计算
									DepthSpacePoint elbowLeftDepthPoint, elbowRightDepthPoint, handLeftDepthPoint, handRightDepthPoint;
									DepthSpacePoint spineShoulderDepthPoint, spineBaseDepthPoint, shoulderLeftDepthPoint, shoulderRightDepthPoint, handTipRightDepthPoint, handTipLeftDepthPoint;
									pCoordinateMapper->MapCameraPointToDepthSpace(joints[JointType_ElbowLeft].Position, &elbowLeftDepthPoint);  //将摄像机坐标系转换为深度空间坐标系
									pCoordinateMapper->MapCameraPointToDepthSpace(joints[JointType_ElbowRight].Position, &elbowRightDepthPoint);
									pCoordinateMapper->MapCameraPointToDepthSpace(joints[JointType_HandLeft].Position, &handLeftDepthPoint);
									pCoordinateMapper->MapCameraPointToDepthSpace(joints[JointType_HandRight].Position, &handRightDepthPoint);

									pCoordinateMapper->MapCameraPointToDepthSpace(joints[JointType_HandTipLeft].Position, &handTipLeftDepthPoint);
									pCoordinateMapper->MapCameraPointToDepthSpace(joints[JointType_HandTipRight].Position, &handTipRightDepthPoint);

									pCoordinateMapper->MapCameraPointToDepthSpace(joints[JointType_SpineShoulder].Position, &spineShoulderDepthPoint);
									pCoordinateMapper->MapCameraPointToDepthSpace(joints[JointType_SpineBase].Position, &spineBaseDepthPoint);
									pCoordinateMapper->MapCameraPointToDepthSpace(joints[JointType_ShoulderLeft].Position, &shoulderLeftDepthPoint);
									pCoordinateMapper->MapCameraPointToDepthSpace(joints[JointType_ShoulderRight].Position, &shoulderRightDepthPoint);

									elbowLeft = Point2f(elbowLeftDepthPoint.Y, elbowLeftDepthPoint.X);   //轴   需要再进行一次坐标对换 因为DepthPoint坐标中第一个存放的是X 但我们第一个需要的是Y
									elbowRight = Point2f(elbowRightDepthPoint.Y, elbowRightDepthPoint.X);
									handLeft = Point2f(handLeftDepthPoint.Y, handLeftDepthPoint.X);      //手部
									handRight = Point2f(handRightDepthPoint.Y, handRightDepthPoint.X);

									//测试模块 手的尖部
									handTipLeft = Point2f(handTipLeftDepthPoint.Y, handTipLeftDepthPoint.X);
									handTipRight = Point2f(handTipRightDepthPoint.Y, handTipRightDepthPoint.X);

									spineShoulderY = spineShoulderDepthPoint.Y; //Y1
									spineBaseY = spineBaseDepthPoint.Y;     //Y2
									shoulderLeftX = shoulderLeftDepthPoint.X;  //X1
									shoulderRightX = shoulderRightDepthPoint.X;	//X2

								}
							}
						}
					}



					//Rect 对象只能用正数整数坐标进行创建 所以需要原坐标进行平移 
					//坐标单位为以摄像头为原点 单位为米 

					Rect torsoROI(Point2f(spineShoulderY, shoulderLeftX), Point2f(spineBaseY, shoulderRightX)); //躯干矩形 坐标点中有负数不好直接定义 一个矩形 低于摄像头就变成负数了  

					if (torsoROI.contains(handLeft)) //全部平移2 保证全为正数 因为Kinect离地不超过2
						isLeftArmSelfOccluded = true;
					if (torsoROI.contains(handRight))
						isRightArmSelfOccluded = true;
					/*cout << "isLeftArmSelfOccluded= " << isLeftArmSelfOccluded << endl;
					cout << "isRightArmSelfOccluded= " << isRightArmSelfOccluded << endl;*/

					isArmSelfOccluded = (isLeftArmSelfOccluded || isRightArmSelfOccluded);
					/*cout << "isArmSelfOccluded= " << isArmSelfOccluded << endl << endl;*/



					//自己定位出来的坐标点
					Point2f elbowRightLoc;  // 右肘
					Point2f elbowLeftLoc;   // 左肘
					Point2f handRightLoc;	 // 右手
					Point2f handLeftLoc;	 // 左手

					Point2f temp1;  //临时的两个节点
					Point2f temp2;
					Point2f temp_1;   //用于两只手发生自遮挡的情况 这两变量用于右手
					Point2f temp_2;


					//找出手臂二值图中的轮廓信息 从而定义出左右手的关节点坐标
					vector<vector<Point>> contours;
					findContours(diff_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

					//根据不同手臂自遮挡状况判断应该进入哪种模式 得到人体骨架关节点
					Mat bodyJunction = bodyDepthBinaryImage.clone(); //将之前预处理过的人体部分二值化图像

					if (isArmSelfOccluded) {

						//轮廓绘制
						vector<vector<Point>> contours;
						findContours(diff_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

						//左右手臂轮廓
						vector<Point> leftArmContour, rightArmContour;

						//左右手骨架像素点坐标
						vector<Point> leftArmSkeletonPoints, rightArmSkeletonPoints;
						vector<Point> tempSkeletonPoints;


						//对手臂进行细化
						Mat skeletonArmImage;
						thinning(diff_image, skeletonArmImage, THINNING_GUOHALL);
						imshow("SkeletonArmImage", skeletonArmImage); //手臂二值图像骨架化操作的结果


						//手部关节点定位 绘制

						//只有左手遮挡
						if (isLeftArmSelfOccluded && !isRightArmSelfOccluded)
						{
							findNonZero(skeletonArmImage, leftArmSkeletonPoints);

							//for (const auto& contour : contours) { //突然发现用不到这个轮廓集合了 直接定位点就行了
							//	leftArmContour.insert(leftArmContour.end(), contour.begin(), contour.end()); //将contour里面的内容复制到leftArmContour
							//}

							if (!leftArmSkeletonPoints.empty()) { //先把手臂骨架线条的两端拿到 此时还不知道节点的层级关系
								temp1 = leftArmSkeletonPoints.front();
								temp2 = leftArmSkeletonPoints.back();
							}


							float dist1 = norm(temp1 - Point2f(elbowLeft.y, elbowLeft.x));  //计算到左肘部的欧几里德距离  L2 范数 先求平方和后求平方根
							float dist2 = norm(temp2 - Point2f(elbowLeft.y, elbowLeft.x));  //cv::Point p(int x, int y); 
																							//Point中x在前，但x在深度图像坐标系下实际是列 使用时需要注意位置

							/*cout << "dist1" << dist1 << endl;
							cout << "dist2" << dist2 << endl;*/

							handLeftLoc = (dist1 > dist2) ? temp1 : temp2;  //手部到肘部的距离更大 所以更远的那个为手部

							line(bodyDepthBinaryImage, handLeftLoc, Point(elbowLeft.y, elbowLeft.x), Scalar(0), 5);

							DrawLine(bodyDepthBinaryImage, joints[JointType_ElbowRight], joints[JointType_HandRight], pCoordinateMapper);
						}


						//只有右手遮挡
						if (isRightArmSelfOccluded && !isLeftArmSelfOccluded)
						{
							findNonZero(skeletonArmImage, rightArmSkeletonPoints);

							if (!rightArmSkeletonPoints.empty()) { //先把手臂骨架线条的两端拿到 此时还不知道节点的层级关系
								temp1 = rightArmSkeletonPoints.front();
								temp2 = rightArmSkeletonPoints.back();
							}

							float dist1 = norm(temp1 - Point2f(elbowRight.y, elbowRight.x));  //计算到右肘部的欧几里德距离
							float dist2 = norm(temp2 - Point2f(elbowRight.y, elbowRight.x));

							//cout << "dist1= " << dist1 << endl;
							//cout << "dist2= " << dist2 << endl;
							//cout << "temp1= " << temp1 << endl;
							//cout << "temp2= " << temp2 << endl;

							handRightLoc = (dist1 > dist2) ? temp1 : temp2;  //手部到肘部的距离更大 所以更远的那个为手部

							//cout << "handRightLoc= " << handRightLoc <<endl; 
							//cout << "elbowRight= " << elbowRight <<endl;

							line(bodyDepthBinaryImage, handRightLoc, Point(elbowRight.y, elbowRight.x), Scalar(0), 5);

							DrawLine(bodyDepthBinaryImage, joints[JointType_ElbowLeft], joints[JointType_HandLeft], pCoordinateMapper);

						}

						//两只手同时遮挡
						if (isLeftArmSelfOccluded && isRightArmSelfOccluded) //换一个思路 
						{

							for (int i = 0; i < contours.size(); i++) {  //遍历两个轮廓 辨别出哪一个是左轮廓 哪一个是右轮廓

								Mat tempContour = Mat::zeros(bodyDepthImage.size(), CV_8UC1); //用来临时存储两个轮廓集中的某一个 以及作为临时骨架细化结果图像

								drawContours(tempContour, contours, i, Scalar(255), FILLED); //将其中一个轮廓提取出来到新图像中 

								thinning(tempContour, tempContour, THINNING_GUOHALL);
								findNonZero(tempContour, tempSkeletonPoints); //临时存储骨架像素点坐标

								if (!tempSkeletonPoints.empty()) {
									temp1 = tempSkeletonPoints.front();
									temp2 = tempSkeletonPoints.back();
								}

								/*cout << "temp1= " << temp1 << endl;
								cout << "temp2= " << temp2 << endl;*/

								float dist1 = norm(temp1 - Point2f(elbowLeft.y, elbowLeft.x));     //计算两个无序坐标点到左肘部的欧几里德距离  
								float dist2 = norm(temp2 - Point2f(elbowLeft.y, elbowLeft.x));

								float dist_1 = norm(temp1 - Point2f(elbowRight.y, elbowRight.x));  //计算到右肘部的欧几里德距离
								float dist_2 = norm(temp2 - Point2f(elbowRight.y, elbowRight.x));


								//当前手臂离哪一个肘部关节点最近 分别计算到左肘的最近距离 以及 到右肘的最近距离
								double distanceLeft = min(dist1, dist2);
								double distanceRight = min(dist_1, dist_2);


								/*cout << "distanceLeft= " << distanceLeft << endl;
								cout << "distanceRight= " << distanceRight << endl;*/

								//离哪一个肘部关节点距离更近 说明该轮廓归属左边或右边
								if (distanceLeft < distanceRight) {
									handLeftLoc = (dist1 > dist2) ? temp1 : temp2;
								}
								else {
									handRightLoc = (dist_1 > dist_2) ? temp1 : temp2;
								}

							}

							line(bodyDepthBinaryImage, handLeftLoc, Point(elbowLeft.y, elbowLeft.x), Scalar(0), 5);
							line(bodyDepthBinaryImage, handRightLoc, Point(elbowRight.y, elbowRight.x), Scalar(0), 5);
						}




						//躯干部分关节点绘制
						GetBodyJunctions(bodyDepthBinaryImage, pBody[userIndex], joints, pCoordinateMapper);
						imshow("BodyJunctions", bodyDepthBinaryImage);

						auto kinectExtractionStart = steady_clock::now();  //记录当前开始的时间戳

						//Kinect识别到的骨架 用于和自定位的骨架进行对比   用于做实验对比
						Mat pBodyImage = bodyDepthBinaryImage.clone();
						GetBodyJunctions(pBodyImage, pBody[userIndex], joints, pCoordinateMapper);
						DrawLine(pBodyImage, joints[JointType_ElbowLeft], joints[JointType_HandLeft], pCoordinateMapper);
						DrawLine(pBodyImage, joints[JointType_ElbowRight], joints[JointType_HandRight], pCoordinateMapper);
						imshow("BodyJunctions_Comparison", pBodyImage);

						auto kinectExtractionEnd = steady_clock::now();  //记录当前开始的时间戳
						if (timer == 500 && frameCount <= 10)  //当间隔1秒时 且采集的图像帧不超过10帧
						{
							auto kinectExtractionDuration = duration_cast<std::chrono::milliseconds>(kinectExtractionEnd - kinectExtractionStart);  //计算持续时间 并且将时间格式改为毫秒
							cout << "第" << frameCount << "帧Kinect骨架提取耗时: " << kinectExtractionDuration.count() << " 毫秒" << endl;
		
						}

					}
					else { //不存在手臂自遮挡 直接调用IBodyFrame接口
						GetBodyJunctions(bodyDepthBinaryImage, pBody[userIndex], joints, pCoordinateMapper);

						//手部绘制 多了一个手腕 看能不能识别出来手腕
						DrawLine(bodyDepthBinaryImage, joints[JointType_ElbowLeft], joints[JointType_HandLeft], pCoordinateMapper);
						DrawLine(bodyDepthBinaryImage, joints[JointType_ElbowRight], joints[JointType_HandRight], pCoordinateMapper);


						imshow("BodyJunctions", bodyDepthBinaryImage);

					}

					//测试模块 人体手部关节点定位
					auto skeletonExtractionEnd = steady_clock::now();  //记录当前开始的时间戳
					if (timer == 500 && frameCount <= 10)  //当间隔1秒时 且采集的图像帧不超过10帧
					{
						auto skeletonExtractionDuration = duration_cast<std::chrono::milliseconds>(skeletonExtractionEnd - skeletonExtractionStart);  //计算持续时间 并且将时间格式改为毫秒
						auto duration = duration_cast<std::chrono::milliseconds>(skeletonExtractionEnd - bodyExtractionStart);  //计算持续时间 并且将时间格式改为毫秒
						cout << "第" << frameCount << "帧人体骨架提取耗时: " << skeletonExtractionDuration.count() << " 毫秒" << endl;
						cout << "第" << frameCount << "帧总提取耗时: " << duration.count() << " 毫秒" << endl;
					}



					Mat colorImage = Mat(colorHeight, colorWidth, CV_8UC4);
					UINT uBufferSize = colorHeight * colorWidth * 4 * sizeof(BYTE);

					//后面为了测试 额外加的获取彩色图像模块
					/*if (pColorReader->AcquireLatestFrame(&pColorFrame) == S_OK) {
						if (pColorFrame->CopyConvertedFrameDataToArray(uBufferSize, colorImage.data, ColorImageFormat_Bgra) == S_OK) {
							imshow("Color Image", colorImage);
						}
						pColorFrame->Release();
					}*/
					

					//测试模块 用于人工标注坐标点

					//ostringstream oss;  //把数字转换为字符串
					//oss << frameCount;
					//string outputPath = "D:/桌面/data/output" ;     // 指定保存图像的目录
					//string filename = "_" + oss.str() + ".jpg" ;    // 指定保存图像的文件名
					//string outputFullPath = outputPath + filename;

					//测试模块 设定每隔1000毫秒 更新一下用于定位时间的一些参数   
					if (timer == 500 && frameCount <= 10) {
						timer = 0;
						
						/*imwrite(outputFullPath, diff_image);
						cout << "保存第" << frameCount << "帧的手臂提取图" << endl;
						cout << "保存第" << frameCount << "帧 Kinect的左手坐标: " << handTipLeft << endl;
						cout << "保存第" << frameCount << "帧 改进算法的左手坐标: " << handLeftLoc << endl << endl;*/


						frameCount++;  //上述操作都已执行完 就可以进入下一帧图像的记录了


					}
					if (timer < 1000 && frameCount <= 10)
					{
						timer += 50;
					}
					if (frameCount == 10) { 
						timer = 0;
						frameCount++;
					}


					
					pBodyFrame->Release();
					pDepthFrame->Release();
					pBodyIndexFrame->Release();

					// 每30ms检测esc键退出循环，否则继续循环 waitKey(30) ==27
					//按下任意按键退出 
					if (waitKey(30) == 27) {
						break;
					}



				}
			}
		}

	}



	//关闭传感器 随后释放前面创建的Reader和pSensor
	pSensor->Close();

	pCoordinateMapper->Release();

	pColorSource->Release();
	pBodyReader->Release();
	pDepthReader->Release();
	pBodyIndexReader->Release();
	//pDepthReader = nullptr;

	pColorReader->Release();
	pBodySource->Release();
	pDepthSource->Release();
	pBodyIndexSource->Release();
	//pDepthSource = nullptr;

	pSensor->Release();
	pSensor = nullptr;
}