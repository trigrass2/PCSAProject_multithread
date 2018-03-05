#pragma once

#include <QWidget>
#include "ui_visualprocessing.h"
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
using  std::cout;
using  std::endl;
using namespace cv;
using namespace HalconCpp;


#define  Pi 3.14159265358979

class VisualProcessing : public QWidget
{
	Q_OBJECT

public:
	VisualProcessing(QWidget *parent = Q_NULLPTR);
	~VisualProcessing();

	//定义机器人位姿结构
	struct RobotPose
	{
		double X;
		double Y;
		double Z;
		double A;//绕Z轴旋转
		double B;//绕Y轴旋转
		double C;//绕X轴旋转
	};


	//初始化lineEdit空间
	void InitializationLineEdit();
	//读取参数文件
	void ReadConfigParameterFile();
	//反馈时间点 格式为yyyy-MM-dd hh-mm-ssPointCloud
	QString ReturnTime();
	//右测头坐标转换到左侧头坐标系下，应用于边缘点云
	Vector<Point3f> RightToLeft(Vector<Point3f> points);
	//右测头坐标转换到左侧头坐标系下，应用于空间圆心
	CoreAlgorithm::StereoCircle RightToLeft(CoreAlgorithm::StereoCircle centerResult);
	//右测头坐标转换到左侧头坐标系下，应用于平面法矢
	CoreAlgorithm::PlaneNormal RightToLeft(CoreAlgorithm::PlaneNormal planeparam);
	//测头坐标系到机器人工具坐标系，应用于边缘点云
	Vector<Point3f> MeasureToRobotTool(Vector<Point3f> points);
	//测头坐标系到机器人工具坐标系，应用于空间圆心
	CoreAlgorithm::StereoCircle MeasureToRobotTool(CoreAlgorithm::StereoCircle centerResult);
	//测头坐标系到机器人工具坐标系，应用于平面法矢
	CoreAlgorithm::PlaneNormal MeasureToRobotTool(CoreAlgorithm::PlaneNormal planeparam);

	//机器人工具坐标系到机器人基座坐标系，应用于边缘点云
	Vector<Point3f> RobotToolToBase(Vector<Point3f> points);
	//机器人工具坐标系到机器人基座坐标系，应用于空间圆心
	CoreAlgorithm::StereoCircle RobotToolToBase(CoreAlgorithm::StereoCircle centerResult);
	//机器人工具坐标系到机器人基座坐标系，应用于平面法矢
	CoreAlgorithm::PlaneNormal RobotToolToBase(CoreAlgorithm::PlaneNormal planeparam);


	//工步一第一部分左侧处理
	void stepOneFirstLeftProcessing(int num,HObject inputImageFirst, HObject inputImageSecond);
	//工步一第一部分右侧处理
	void stepOneSecondRightProcessing(int num,HObject inputImageFirst, HObject inputImageSecond);
	//工步二左侧处理
	void stepTwoLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//工步二右侧处理
	void stepTwoRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//工步三左侧处理
	void stepThreeLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//工步三右侧处理
	void stepThreeRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//工步二左侧法兰孔处理
	void stepTwoLeftFlangeHoleProcessing(int num, HObject inputImage);
	//工步二右侧法兰孔处理
	void stepTwoRightFlangeHoleProcessing(int num, HObject inputImage);
	//工步三左侧法兰孔处理
	void stepThreeLeftFlangeHoleProcessing(int num, HObject inputImage);
	//工步三右侧法兰孔处理
	void stepThreeRightFlangeHoleProcessing(int num, HObject inputImage);
	//工步四左侧处理
	void stepFourLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//工步四右侧处理
	void stepFourRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//工步五左侧处理
	void stepFiveLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//工步五右侧处理
	void stepFiveRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//角度制转弧度制
	double deg2rad(double deg);
	//计算从机器人工具坐标系到机器人基座坐标系的变换矩阵
	void calculateRobotToolToBaseMatrix(RobotPose robotPose);
	//计算导轨移动距离
	void calculateMoveDistance(int num, double& firstDistance, double &secondDistance);
	//获取约束半径
	void getRestrictRadius(int num, double &R);


	//lineEdit控件组合
	const int lineEditNum = 124;
	QLineEdit* lineEdit[124];
	//创建文件夹命名
	static QString engineNumFileName;
	
	//声明机器人位姿
	static RobotPose robotPose;
	//声明从机器人工具坐标系到机器人基座坐标系变换矩阵
	static Mat robotToolToBaseMatrix;
	//光栅尺反馈导轨移动距离
	vector<double> leftRunDis;
	vector<double> rightRunDis;

signals:
	void sendProcessingResult(bool planeparamStatus/*平面法矢处理状态*/,
							CoreAlgorithm::PlaneNormal planeparam/*平面法矢*/,
							CoreAlgorithm::StereoCircle centerResult/*空间圆心*/,
							pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr/*边缘点云 PCL格式*/,
							int num/*num = 1 表示传输左侧测量参数；num = 2 表示传输右侧测量参数*/);
	void sendStepParam(double CameraNoScanSpeed, double CameraScanDistance, double CameraFirstNoScanDistance, double CameraSecondNoScanDistance);
public slots:
	//更改相关数据
	void on_updataButton_clicked();
	//关闭使能
	void on_setButton_clicked();
	//接收工步序号，反馈相关参数；
	void receiveStepNum(int num);
	//接收处理工位号及其图片
	void receiveInformation(int num, HObject inputImageFirst, HObject inputImageSecond);
	//接收光删尺数据
	void receiveLeftGratingData(vector<double>left);
	void receiveRightGratingData(vector<double>right);
	//接收机器人位姿
	void receiveRobotPose(RobotPose robotPo);
private:
	Ui::VisualProcessing ui;
	CoreAlgorithm *alg;

	
};

