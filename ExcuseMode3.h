#pragma once
#include "ExecuseMode2.h"

class ExcuseMode3 :public QThread
{
	Q_OBJECT
protected:
	void run();

public:
	void execusemode2(TYcamera& a, RobotControl& b, chooseblack& c, complexIm& d);

	//д��2021.7.8 �µ�ִ�г���
	void NewExecusemode(TYcamera& a, RobotControl& b, chooseblack& c, complexIm& d);


public:
	bool WorkProcess = true;	//һ������ͣ�õı���������Ϊtrueʱ����ڹ���״̬����֮��ͣ
	std::vector<int> Robx;
	std::vector<int> Roby;
	std::vector<int> Robz;
	std::vector<int> Robwt1;
	std::vector<int> Robwt2;
	cv::Mat ColorImage;   //��Ż���У�������ͼ��		
	cv::Mat DepthImage;   //������ͼ��
	cv::Mat ROIColorImage;   //��Ų�ɫͼ�����Ȥ����
	cv::Mat ROIDepthImage;	//������ͼ�����Ȥ����
	std::vector<float> depthData;    //���ÿ�����������Ϣ
	std::vector<float> Radiuses;	//���ͼ���й����뾶
	std::vector<cv::Point2f> imcenters;		//���ͼ���й������ĵ�����		
	std::vector<cv::Point2f> robcenters;		//��Ż���������ϵ�������ĵ�����		
	std::vector<cv::Mat> BinaryIms;		//��Ÿ�ʴ���ͺ�Ķ�ֵͼ��
	cv::Rect tu;	//��׼Rect
	cv::Rect tColor;	//��ɫͼ���Rect
	cv::Rect tDepth;	//���ͼ���Rect
	float NeedZ0 = 1290;	//��׼Zֵ
	float MaxWorkZ = 2450;	//�����Zֵ���������ֵ��������
	float FirstPaperZ; //�����⵽��ֽƤZֵ


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

