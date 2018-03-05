#ifndef COREALGORITHM_H
#define COREALGORITHM_H

#include <QObject>
#include<iostream>
#include <fstream>
#include"math.h"
#include <iomanip>
////PCL相关头文件
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件  
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件  
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/search/kdtree.h>  
#include <pcl/filters/passthrough.h>  
#include <pcl/filters/extract_indices.h>  
//
#include <pcl/features/boundary.h>
#include <pcl/features/integral_image_normal.h>  //法线估计类头文件
#include <pcl/features/normal_3d.h>

#include <pcl/range_image/range_image.h>
#include <pcl/io/png_io.h>
#include <pcl/visualization/common/float_image_utils.h>

//绘制空间球体
#include <boost/thread/thread.hpp>  
#include <pcl/common/common_headers.h>  
#include <pcl/features/normal_3d.h>  

//10.18平面拟合
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
//QT 随机数
#include<QTime>
//添加OpenCV
#include "cv.h"
#include "opencv.hpp"
//添加halcon
#include "HalconCpp.h"
//测试
#include<QFile>
#include <QTextStream>

//空间域名
using  std::cout;
using  std::endl;
using namespace cv;
using namespace HalconCpp;

class CoreAlgorithm : public QObject
{
	Q_OBJECT


public:
	CoreAlgorithm(QObject *parent);
	~CoreAlgorithm();
	//定义平面法矢数据结构
	struct PlaneNormal
	{
		double A;//法向量x
		double B;//法向量y
		double C;//法向量z
		double D;//常数
	};
	//定义空间圆数据结构
	struct StereoCircle
	{
		Point3f NormalVector;//平面法矢
		Point3f center;//空间圆心坐标
		bool status;//求解圆心是否成功
	};
	//深度图标定，转换为世界坐标系下点
	Vector<Point3f> depthMapCalibration(HObject &inputImage,bool status);
	//从深度图中获取平面
	HObject getPlaneImage(HObject &inputImage, bool& status);
	//从深度图中获取发送机花键边缘
	HObject getEngineSplineEdgeImage(HObject &inputImage, bool& status);
	//从深度图中获取前传动箱花键边缘
	HObject getfrontDriveBoxSplineEdgeImage(HObject &inputImage, bool& status);
	//从深度图中获取止口边缘
	HObject getFlangeEdgeImage(HObject &inputImage, bool& status);
	//从深度图中获取法兰孔边缘
	HObject getFlangeHoleEdgeImage(HObject &inputImage, bool& status);
	//获取平面法矢
	PlaneNormal getPlaneNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source);
	//计算边缘点到拟合空间平面的距离，并设置距离阈值获取符合阈值的点
	void setDistanceThresholdAndGetPoints(Vector<Point3f>&inputPoints,//输入点
										  PlaneNormal planeparam,//输入平面法矢信息
										  Vector<double>distance,//计算边缘点到拟合空间平面的距离
										  double lowThreshold,//下阈值
										  double highThreshold,//上阈值
										  Vector<Point3f>&outputPoints//输出点
		);
	//计算边缘投影点
	Vector<Point3f> ComputedEdgeProjectionPoints(Vector<Point3f> &points, PlaneNormal&planeparam);
	//基于半径约束的空间圆拟合
	StereoCircle fitSteroCircle(Vector<Point3f> &subpoints, PlaneNormal&planeparam, double R);
	//Ransac算法
	vector<Point2f> RansacAlgorithm(vector<Point2f> &SourcePoint, double Radius);
	//约束半径的平面圆拟合
	Point2f fitPlaneCircleBaseOnRadius(vector<Point2f>points, double R);
	//将三维坐标赋值给PCL点云坐标
	void threeDimensionToPCLPoints(Vector<Point3f>&points, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints);
	//显示点云
	void displayCloude(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_ptr);
	//测试使用 生成点云
	Vector<Point3f> createCloud(HObject &inputImage);
	//右测头坐标转换到左侧头坐标系下
	Vector<Point3f> RightToLeft(Vector<Point3f> points);
	//计算边缘点到圆心的距离，并保存信息,返回平均距离
	double CalculateBorderPointsToCenterAverageDistance(Vector<Point3f> points, StereoCircle centerPoint);
	//计算边缘点到圆心的距离
	void CalculateBorderPointsToCenterDistance(Vector<Point3f> points, StereoCircle centerPoint, Vector<double> &distances);
	//设置边缘点到圆心的距离阈值，删除到圆心过远和过近点
	Vector<Point3f> filterBorderPointsOnDistanceThrehold(Vector<Point3f> borderPoints, Vector<double> distances, double R,double threholdRange);

private:
	//halcon内部自定义函数
	void ReadC5CSCalibrationFile(HTuple hv_CalibFileName, HTuple *hv_Homography, HTuple *hv_gFactor,
		HTuple *hv_NumCoefficients, HTuple *hv_Tinv1, HTuple *hv_Tinv2);
	void PolynomialCorrection(HTuple hv_X_, HTuple hv_Z_, HTuple hv_NumCoefficients,
		HTuple hv_Tinv1, HTuple hv_Tinv2, HTuple *hv_X_NEW, HTuple *hv_Z_NEW);
};

#endif // COREALGORITHM_H
