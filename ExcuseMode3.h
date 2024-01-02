#pragma once
#include "ExecuseMode2.h"

class ExcuseMode3 :public QThread
{
	Q_OBJECT
protected:
	void run();

public:
	void execusemode2(TYcamera& a, RobotControl& b, chooseblack& c, complexIm& d);

	//写于2021.7.8 新的执行程序
	void NewExecusemode(TYcamera& a, RobotControl& b, chooseblack& c, complexIm& d);


public:
	bool WorkProcess = true;	//一个做暂停用的变量，当其为true时则出于工作状态，反之暂停
	std::vector<int> Robx;
	std::vector<int> Roby;
	std::vector<int> Robz;
	std::vector<int> Robwt1;
	std::vector<int> Robwt2;
	cv::Mat ColorImage;   //存放畸变校正过后的图像		
	cv::Mat DepthImage;   //存放深度图像
	cv::Mat ROIColorImage;   //存放彩色图像感兴趣区域
	cv::Mat ROIDepthImage;	//存放深度图像感兴趣区域
	std::vector<float> depthData;    //存放每个工件深度信息
	std::vector<float> Radiuses;	//存放图像中工件半径
	std::vector<cv::Point2f> imcenters;		//存放图像中工件中心的坐标		
	std::vector<cv::Point2f> robcenters;		//存放机器人坐标系工件中心的坐标		
	std::vector<cv::Mat> BinaryIms;		//存放腐蚀膨胀后的二值图像
	cv::Rect tu;	//基准Rect
	cv::Rect tColor;	//彩色图像的Rect
	cv::Rect tDepth;	//深度图像的Rect
	float NeedZ0 = 1290;	//基准Z值
	float MaxWorkZ = 2450;	//最大工作Z值，超过这个值则码垛结束
	float FirstPaperZ; //相机检测到的纸皮Z值


	TYcamera a;
	RobotControl b;
	chooseblack c;
	complexIm d;
signals:
	void StartExcuse();
	void DisplayColor();
	void DisplayBinary();
	void WorkState(char* str);
	void DisplayLayerNumbers(QString str);
	void DisplayRobX(QString str);
	void DisplayRobY(QString str);
	void DisplayRobZ(QString str);
	void DisplayPieceCount(QString str);

};

