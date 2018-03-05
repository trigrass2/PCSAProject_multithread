#pragma once

#include <QThread>
#include <QMutex> 
#include <iostream>
#include<fstream>
//添加halcon
#include "HalconCpp.h"
//添加OpenCV
#include "cv.h"
#include "opencv.hpp"
//添加核心算法
#include "corealgorithm.h"
#include<QFile>
#include <QTimer>
#include <QDateTime>
#include <QTextStream>
#include<QMessageBox>

//
#include "define.h"
//视觉处理类头文件
#include "visualprocessing.h"
//核心算法类头文件
#include "corealgorithm.h"
using  std::cout;
using  std::endl;
using namespace cv;
using namespace HalconCpp;


#define  Pi 3.14159265358979

class ProcessThread : public QThread
{
	Q_OBJECT

public:
	ProcessThread(QObject *parent);
	~ProcessThread();

	void run();

	bool threadStatus;
	int stepNum;

	//左侧头运行
	void LeftScannerRun(int num, pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult);
	//右测头运行
	void RightScannerRun(int num, pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult);
	//工步1第一部分
	void stepOneFirst();


	
	//工步一第一部分左侧处理
	void stepOneFirstLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//工步一第一部分右侧处理
	void stepOneSecondRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);









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

	//调试使用，获取图片序号
	int imageNum;
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
private:
	VisualProcessing *vp;
	CoreAlgorithm *alg;
	QMutex mutex;
	
};
