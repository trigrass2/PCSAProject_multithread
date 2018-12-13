#pragma once

//运动控制头文件
#include "motioncontrol.h"

#include <QThread>
#include <QMutex> 
#include <iostream>
#include<fstream>
//添加halcon
#include "HalconCpp.h"
//添加OpenCV
#include "cv.h"
#include "opencv.hpp"

#include<QFile>
#include <QTimer>
#include <QDateTime>
#include <QTextStream>
#include<QMessageBox>

//全局变量
#include "define.h"
//视觉处理类头文件
#include "visualprocessing.h"
//添加核心算法
#include "corealgorithm.h"
//相机采集
#include "cameracapture.h"
//串口类
#include "serialcommunication.h"

using  std::cout;
using  std::endl;
using namespace cv;
using namespace HalconCpp;


#define  Pi 3.14159265358979

class ProcessThread : public QThread
{
	Q_OBJECT

public:
	ProcessThread();
	~ProcessThread();
	void initialization();
	void run();

	bool threadStatus;
	//int stepNum;
	stepName stepNum;
	//左侧头运行
	void LeftScannerRun(stepName stepNum, pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult);
	//右测头运行
	void RightScannerRun(stepName stepNum, pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult);
	//工步
	void step();
	
	/**
	*在实际测量中，由现场情况确定发动机、前传动箱以及综合传动箱在左，在右；
	*/
	//定义边缘点点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr leftCircle_cloudPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr rightCircle_cloudPoints;
	//定义空间圆心
	CoreAlgorithm::StereoCircle leftCenterResult;
	CoreAlgorithm::StereoCircle rightCenterResult;
	//定义平面法矢
	CoreAlgorithm::PlaneNormal leftPlaneNormal;
	CoreAlgorithm::PlaneNormal rightPlaneNormal;
	//声明处理结果 只要法矢拟合成功即为true
	bool leftProcessingStatus;
	bool rightProcessingStatus;
	//定义法兰孔边缘点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr leftFlangeHoleCircle_cloudPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr rightFlangeHoleCircle_cloudPoints;
	//定义法兰孔空间圆心
	CoreAlgorithm::StereoCircle leftFlangeHoleCenterResult;
	CoreAlgorithm::StereoCircle rightFlangeHoleCenterResult;
	/**
	*为每个工位单独声明变量
	*/
	//应用于工步一第二部分 前传动箱或综合传动箱数据
	//第一次
	//定义边缘点点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr transmissionCaseCircleFirst_cloudPoints;
	//定义空间圆心
	CoreAlgorithm::StereoCircle transmissionCaseCenterResultFirst;
	//第二次
	//定义边缘点点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr transmissionCaseCircleSecond_cloudPoints;
	//定义空间圆心
	CoreAlgorithm::StereoCircle transmissionCaseCenterResultSecond;
	//定义两次测量圆心的方向矢量
	Point3f directionVector;

	//定义工步参数
	double CameraNoScanSpeed;//测头非扫描时运动速度
	double CameraScanDistance;//测头扫描时运动距离
	double CameraFirstNoScanDistance;//测头运动到第一测量位置的运动距离
	double CameraSecondNoScanDistance;//测头运动到第二测量位置的运动距离
	//光栅尺反馈导轨移动距离
	vector<double> leftRunDis;
	vector<double> rightRunDis;
	//调试使用，获取图片序号
	int imageNum;

	HObject inputImageFirst;
	HObject inputImageSecond;
public:
	//返回每个工步采集方位信息
	/*stepCapture *getStepCapture(stepName name);*/

	//从深度图中获取平面
	HObject getPlaneImage(HObject &inputImage, bool& status);

	//H101 检测发动机止口法矢
	void stepOneFirstLeftProcessing(int num, HObject &inputImageFirst, HObject &inputImageSecond);
	//H102 检测前传动箱止口面圆心1
	void stepOneSecondRightProcessing(int num, HObject &inputImageFirst, HObject &inputImageSecond);
	//H103 检测前传动箱止口面圆心2，圆心1，2相减1

	//H201 检测发动机止口平面法矢和圆心坐标
	void stepTwoLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//H202 检测前传动箱止口平面法矢和圆心坐标
	void stepTwoRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//H203 检测发动机止口一个法兰孔圆心坐标
	void stepTwoLeftFlangeHoleProcessing(int num, HObject inputImage);
	//H204 检测前传动箱止口一个法兰孔圆心坐标
	void stepTwoRightFlangeHoleProcessing(int num, HObject inputImage);
	//H301 检测综合传送箱止口平面法矢和圆心坐标
	void stepThreeLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//H302 检测前传动箱止口平面法矢和圆心坐标
	void stepThreeRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//H303 检测综合传送箱止口一个法兰孔圆心坐标
	void stepThreeLeftFlangeHoleProcessing(int num, HObject inputImage);
	//H304 检测前传动箱止口一个法兰孔圆心坐标
	void stepThreeRightFlangeHoleProcessing(int num, HObject inputImage);
	//H401 检测综传动箱花键孔的平面法矢和圆心 
	void stepFourLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//H402 检测前传动箱花键轴的平面法矢和圆心 
	void stepFourRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//H403 检测综传动箱花键相位角

	//H404 检测前传动箱花键相位角

	//H501 检测发动机花键孔的平面法矢和圆心
	void stepFiveLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//H502 检测前传动箱花键轴的平面法矢和圆心
	void stepFiveRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//H503 检测发动机花键相位角

	//H504 检测前传动箱花键相位角



	//纵置
	void v_stepThreeFirstLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	void v_stepThreeFirstRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	void v_stepThreeSecondLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	void v_stepThreeSecondRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
signals:
	//发送状态更新
	void sendStatusUpdate(QString str,int num);
	//发送请求反馈工位导轨移动量及速度信息
	void sendStepNum(int num);
	//显示图片并保存
	void sendImageAndSave(HObject image, int num, QString name);
	//发送处理工位序号及其图片
	void sendProcessingInformation(int num, HObject inputImageFirst, HObject inputImageSecond);
	//发送处理结果
	void sendProcessingResult(bool planeparamStatus/*平面法矢处理状态*/,
		CoreAlgorithm::PlaneNormal planeparam/*平面法矢*/,
		CoreAlgorithm::StereoCircle centerResult/*空间圆心*/,
		pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr/*边缘点云 PCL格式*/,
		int num/*num = 1 表示传输左侧测量参数；num = 2 表示传输右侧测量参数*/);

	//调试使用，获取图片序号
	void sendImageNum();
	void sendLeftGratingData(vector<double> leftRunDis);
	//发送右侧采集时，导轨实际运动量
	void sendRightGratingData(vector<double> rightRunDis);
	//结束线程
	void endThread();
public slots:
	//接收VisualProcessing UI的参数
	void receiveStepParam(double CamNoScanSpeed, double CamScanDistance, double CamFirstNoScanDistance, double CamSecondNoScanDistance);
	//接收视觉处理结果，用于显示
	void receiveProcessingResult(bool planeparamStatus/*平面法矢处理状态*/,
		CoreAlgorithm::PlaneNormal planeparam/*平面法矢*/,
		CoreAlgorithm::StereoCircle centerResult/*空间圆心*/,
		pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr/*边缘点云 PCL格式*/,
		int num/*工步号*/);
	/*num = 1 表示传输左侧测量参数；num = 2 表示传输右侧测量参数；num = 3 表示传输左侧法兰孔测量参数；num = 4 表示传输右侧法兰孔测量参数*/


	//调试使用，获取图片序号
	void receiveImageNum(int num);

	//打开运动控制
	void motionActionSlot();
	//打开相机设置
	void cameraActionSlot();
private:
	VisualProcessing *vp;
	CoreAlgorithm *alg;
	QMutex mutex;
	MotionControl *motion;
	CameraCapture *cap;
	SerialCommunication *serial;
};
