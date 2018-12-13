#ifndef DISPLAYGUI_H
#define DISPLAYGUI_H
#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

#include <QtWidgets/QMainWindow>
#include<QString>
#include "ui_displaygui.h"
//处理线程类头文件 
#include "processthread.h"
//
#include "define.h"
//网口通讯头文件
#include "networkcommunication.h"
//核心算法头文件
#include "corealgorithm.h"
//视觉处理类头文件
#include "visualprocessing.h"

#include<iostream>
#include <QTimer>
#include <QDateTime>
#include <QString>
#include <QByteArray>
#include<QMessageBox>
#include<QFile>
#include <QTextStream>
#include <QProcess>
#include <QDir>
// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include<QVTKWidget.h>

#include "HalconCpp.h"

using namespace HalconCpp;

class DisplayGUI : public QMainWindow
{
	Q_OBJECT

public:
	DisplayGUI(QWidget *parent = 0);
	~DisplayGUI();
	


	//定义函数
	//显示点云
	//左侧圆心为红色，右侧为绿色
	void displayCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr LeftCloud_ptr/*左侧点云*/, CoreAlgorithm::StereoCircle leftCenterResult/*左侧圆心*/,
		pcl::PointCloud<pcl::PointXYZ>::Ptr RightCloud_ptr/*右侧点云*/, CoreAlgorithm::StereoCircle RightCenterResult/*右侧圆心*/);
	//函数重载 显示工步2 工步3点云
	//法兰孔左侧圆心为蓝色，右侧为绿色
	void displayCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr LeftCloud_ptr/*左侧点云*/, 
		CoreAlgorithm::StereoCircle leftCenterResult/*左侧圆心*/,
		pcl::PointCloud<pcl::PointXYZ>::Ptr RightCloud_ptr/*右侧点云*/,
		CoreAlgorithm::StereoCircle RightCenterResult/*右侧圆心*/,
		pcl::PointCloud<pcl::PointXYZ>::Ptr LeftFlangeHoleCloud_ptr/*左侧法兰孔点云*/,
		CoreAlgorithm::StereoCircle leftFlangeHoleCenterResult/*左侧法兰孔圆心*/,
		pcl::PointCloud<pcl::PointXYZ>::Ptr RightFlangeHoleCloud_ptr/*右侧法兰孔点云*/,
		CoreAlgorithm::StereoCircle RightFlangeHoleCenterResult/*右侧法兰孔圆心*/);
	//只显示一个法兰孔时，圆心为蓝色
	void displayCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr LeftCloud_ptr/*左侧点云*/,
		CoreAlgorithm::StereoCircle leftCenterResult/*左侧圆心*/,
		pcl::PointCloud<pcl::PointXYZ>::Ptr RightCloud_ptr/*右侧点云*/,
		CoreAlgorithm::StereoCircle RightCenterResult/*右侧圆心*/,
		pcl::PointCloud<pcl::PointXYZ>::Ptr FlangeHoleCloud_ptr/*某侧法兰孔点云*/,
		CoreAlgorithm::StereoCircle FlangeHoleCenterResult/*某侧法兰孔圆心*/);
	void displayCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_ptr, CoreAlgorithm::StereoCircle centerResult);
	//void displayCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudFirst_ptr, CoreAlgorithm::StereoCircle centerResultFirst);
	//显示图片
	void displayImage(HObject Image,int num);

	/**
	*纵置发动机
	*/
	
	//HObject转化为Mat类型
	Mat HImageToIplImage(HObject &Hobj);
	//Mat转化为QImage类型
	QImage cvMat2QImage(const cv::Mat& mat);
	//初始化部件
	void initialization();
	//左侧头运行
	void LeftScannerRun(int num,pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult);
	//右测头运行
	void RightScannerRun(int num,pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult);
	//初始化lineEdit空间
	//void InitializationLineEdit();
	//读取参数文件
	//void ReadConfigParameterFile();
	//反馈时间点 格式为yyyy-MM-dd hh-mm-ss
	QString ReturnTime();
	//保存图片
	void SaveImage(HObject Image,QString name);
	//分解工控机发送的信号
	void resolveSignal(QString info);
	//保存日志文件信息
	void saveInformationToJournalFile(QString info);
	//创建工程所需要的各个文件夹
	void createProjectDirectory(QString info);
	//创建工程数据保存文件，同时生成参数名
	void createParamFirle(QString info);
	//定义变量
	//Socket接受数据
	QString engineVersion;
	int socketNum;
	//lineEdit控件组合
	//QLineEdit* lineEdit[72];

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

	/**
	*横置发动机参数
	*/



	//创建文件夹命名
	QString engineNumFileName;
	//定义工步参数
	double CameraNoScanSpeed;//测头非扫描时运动速度
	double CameraScanDistance;//测头扫描时运动距离
	double CameraFirstNoScanDistance;//测头运动到第一测量位置的运动距离
	double CameraSecondNoScanDistance;//测头运动到第二测量位置的运动距离

	//光栅尺反馈导轨移动距离
	vector<double> leftRunDis;
	vector<double> rightRunDis;
	//发送结果信息
	ResultMessage resultMessage;
signals:
	//更新状态栏信息
	void statusUpdate(QString,int);
	//发送处理工位序号及其图片
	void sendProcessingInformation(int num, HObject inputImageFirst, HObject inputImageSecond);
	//发送请求反馈工位导轨移动量及速度信息
	void sendStepNum(int num);
	//发送左侧采集时，导轨实际运动量
	void sendLeftGratingData(vector<double> leftRunDis);
	//发送右侧采集时，导轨实际运动量
	void sendRightGratingData(vector<double> rightRunDis);
	//发送机器人位姿
	void sendRobotPose(VisualProcessing::RobotPose robotPose);

	//测试使用，发送图片序号
	void sendImageNum(int num);
public slots:
	//打开网口通讯设置
	void networkActionSlot();
	
	//打开手眼标定程序
	void eyeHandActionSlot();
	//打开视觉处理参数设定
	void visualProcessingActionSlot();
	//接受网口数据，进行工步操作
	void receiveSocket(QString info);
	//显示历史数据
	void displayDataActionSlot();
	//更新状态信息
	void updateStatusInformation(QString Info,int num);
	//更改相关数据
	//void on_updataButton_clicked();
	//关闭使能
	//void on_setButton_clicked();
	//接收视觉处理结果，用于显示
	void receiveProcessingResult(bool planeparamStatus/*平面法矢处理状态*/,
		CoreAlgorithm::PlaneNormal planeparam/*平面法矢*/,
		CoreAlgorithm::StereoCircle centerResult/*空间圆心*/,
		pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr/*边缘点云 PCL格式*/,
		int num/*工步号*/);
	/*num = 1 表示传输左侧测量参数；num = 2 表示传输右侧测量参数；num = 3 表示传输左侧法兰孔测量参数；num = 4 表示传输右侧法兰孔测量参数*/
	//接收VisualProcessing UI的参数
	void receiveStepParam(double CamNoScanSpeed, double CamScanDistance, double CamFirstNoScanDistance, double CamSecondNoScanDistance);
	//显示图片并保存
	void receiveImageAndSave(HObject image, int num, QString name);

	//测试使用，发送图片序号
	void receiveImageNum();

	//结束线程
	void endThread();
private:
	Ui::DisplayGUIClass ui;
	NetworkCommunication *socket;
	CoreAlgorithm *alg;
	VisualProcessing *vp;
	ProcessThread *proThread;
	//PCL显示
	pcl::visualization::PCLVisualizer::Ptr viewer_1;
	pcl::visualization::PCLVisualizer::Ptr viewer_2;
};

#endif // DISPLAYGUI_H
