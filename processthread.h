#pragma once

#include <QThread>
#include <QMutex> 
#include <iostream>
#include<fstream>
//���halcon
#include "HalconCpp.h"
//���OpenCV
#include "cv.h"
#include "opencv.hpp"
//��Ӻ����㷨
#include "corealgorithm.h"
#include<QFile>
#include <QTimer>
#include <QDateTime>
#include <QTextStream>
#include<QMessageBox>

//
#include "define.h"
//�Ӿ�������ͷ�ļ�
#include "visualprocessing.h"
//�����㷨��ͷ�ļ�
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

	//���ͷ����
	void LeftScannerRun(int num, pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult);
	//�Ҳ�ͷ����
	void RightScannerRun(int num, pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult);
	//����1��һ����
	void stepOneFirst();


	
	//����һ��һ������ദ��
	void stepOneFirstLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//����һ��һ�����Ҳദ��
	void stepOneSecondRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);









	/**
	*��ʵ�ʲ����У����ֳ����ȷ����������ǰ�������Լ��ۺϴ������������ң�
	*/
	//�����Ե�����
	pcl::PointCloud<pcl::PointXYZ>::Ptr leftCircle_cloudPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr rightCircle_cloudPoints;
	//����ռ�Բ��
	CoreAlgorithm::StereoCircle leftCenterResult;
	CoreAlgorithm::StereoCircle rightCenterResult;
	//����ƽ�淨ʸ
	CoreAlgorithm::PlaneNormal leftPlaneNormal;
	CoreAlgorithm::PlaneNormal rightPlaneNormal;
	//���������� ֻҪ��ʸ��ϳɹ���Ϊtrue
	bool leftProcessingStatus;
	bool rightProcessingStatus;
	//���巨���ױ�Ե����
	pcl::PointCloud<pcl::PointXYZ>::Ptr leftFlangeHoleCircle_cloudPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr rightFlangeHoleCircle_cloudPoints;
	//���巨���׿ռ�Բ��
	CoreAlgorithm::StereoCircle leftFlangeHoleCenterResult;
	CoreAlgorithm::StereoCircle rightFlangeHoleCenterResult;
	/**
	*Ϊÿ����λ������������
	*/
	//Ӧ���ڹ���һ�ڶ����� ǰ��������ۺϴ���������
	//��һ��
	//�����Ե�����
	pcl::PointCloud<pcl::PointXYZ>::Ptr transmissionCaseCircleFirst_cloudPoints;
	//����ռ�Բ��
	CoreAlgorithm::StereoCircle transmissionCaseCenterResultFirst;
	//�ڶ���
	//�����Ե�����
	pcl::PointCloud<pcl::PointXYZ>::Ptr transmissionCaseCircleSecond_cloudPoints;
	//����ռ�Բ��
	CoreAlgorithm::StereoCircle transmissionCaseCenterResultSecond;
	//�������β���Բ�ĵķ���ʸ��
	Point3f directionVector;

	//���幤������
	double CameraNoScanSpeed;//��ͷ��ɨ��ʱ�˶��ٶ�
	double CameraScanDistance;//��ͷɨ��ʱ�˶�����
	double CameraFirstNoScanDistance;//��ͷ�˶�����һ����λ�õ��˶�����
	double CameraSecondNoScanDistance;//��ͷ�˶����ڶ�����λ�õ��˶�����

	//����ʹ�ã���ȡͼƬ���
	int imageNum;
signals:
	//����״̬����
	void sendStatusUpdate(QString str,int num);
	//������������λ�����ƶ������ٶ���Ϣ
	void sendStepNum(int num);
	//��ʾͼƬ������
	void sendImageAndSave(HObject image, int num, QString name);
	//���ʹ���λ��ż���ͼƬ
	void sendProcessingInformation(int num, HObject inputImageFirst, HObject inputImageSecond);
	//���ʹ�����
	void sendProcessingResult(bool planeparamStatus/*ƽ�淨ʸ����״̬*/,
		CoreAlgorithm::PlaneNormal planeparam/*ƽ�淨ʸ*/,
		CoreAlgorithm::StereoCircle centerResult/*�ռ�Բ��*/,
		pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr/*��Ե���� PCL��ʽ*/,
		int num/*num = 1 ��ʾ����������������num = 2 ��ʾ�����Ҳ��������*/);

	//����ʹ�ã���ȡͼƬ���
	void sendImageNum();


public slots:
	//����VisualProcessing UI�Ĳ���
	void receiveStepParam(double CamNoScanSpeed, double CamScanDistance, double CamFirstNoScanDistance, double CamSecondNoScanDistance);
	//�����Ӿ���������������ʾ
	void receiveProcessingResult(bool planeparamStatus/*ƽ�淨ʸ����״̬*/,
		CoreAlgorithm::PlaneNormal planeparam/*ƽ�淨ʸ*/,
		CoreAlgorithm::StereoCircle centerResult/*�ռ�Բ��*/,
		pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr/*��Ե���� PCL��ʽ*/,
		int num/*������*/);
	/*num = 1 ��ʾ����������������num = 2 ��ʾ�����Ҳ����������num = 3 ��ʾ������෨���ײ���������num = 4 ��ʾ�����Ҳ෨���ײ�������*/


	//����ʹ�ã���ȡͼƬ���
	void receiveImageNum(int num);
private:
	VisualProcessing *vp;
	CoreAlgorithm *alg;
	QMutex mutex;
	
};
