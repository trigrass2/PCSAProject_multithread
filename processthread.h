#pragma once

//�˶�����ͷ�ļ�
#include "motioncontrol.h"

#include <QThread>
#include <QMutex> 
#include <iostream>
#include<fstream>
//���halcon
#include "HalconCpp.h"
//���OpenCV
#include "cv.h"
#include "opencv.hpp"

#include<QFile>
#include <QTimer>
#include <QDateTime>
#include <QTextStream>
#include<QMessageBox>

//ȫ�ֱ���
#include "define.h"
//�Ӿ�������ͷ�ļ�
#include "visualprocessing.h"
//��Ӻ����㷨
#include "corealgorithm.h"
//����ɼ�
#include "cameracapture.h"
//������
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
	//���ͷ����
	void LeftScannerRun(stepName stepNum, pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult);
	//�Ҳ�ͷ����
	void RightScannerRun(stepName stepNum, pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult);
	//����
	void step();
	
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
	//��դ�߷��������ƶ�����
	vector<double> leftRunDis;
	vector<double> rightRunDis;
	//����ʹ�ã���ȡͼƬ���
	int imageNum;

	HObject inputImageFirst;
	HObject inputImageSecond;
public:
	//����ÿ�������ɼ���λ��Ϣ
	/*stepCapture *getStepCapture(stepName name);*/

	//�����ͼ�л�ȡƽ��
	HObject getPlaneImage(HObject &inputImage, bool& status);

	//H101 ��ⷢ����ֹ�ڷ�ʸ
	void stepOneFirstLeftProcessing(int num, HObject &inputImageFirst, HObject &inputImageSecond);
	//H102 ���ǰ������ֹ����Բ��1
	void stepOneSecondRightProcessing(int num, HObject &inputImageFirst, HObject &inputImageSecond);
	//H103 ���ǰ������ֹ����Բ��2��Բ��1��2���1

	//H201 ��ⷢ����ֹ��ƽ�淨ʸ��Բ������
	void stepTwoLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//H202 ���ǰ������ֹ��ƽ�淨ʸ��Բ������
	void stepTwoRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//H203 ��ⷢ����ֹ��һ��������Բ������
	void stepTwoLeftFlangeHoleProcessing(int num, HObject inputImage);
	//H204 ���ǰ������ֹ��һ��������Բ������
	void stepTwoRightFlangeHoleProcessing(int num, HObject inputImage);
	//H301 ����ۺϴ�����ֹ��ƽ�淨ʸ��Բ������
	void stepThreeLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//H302 ���ǰ������ֹ��ƽ�淨ʸ��Բ������
	void stepThreeRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//H303 ����ۺϴ�����ֹ��һ��������Բ������
	void stepThreeLeftFlangeHoleProcessing(int num, HObject inputImage);
	//H304 ���ǰ������ֹ��һ��������Բ������
	void stepThreeRightFlangeHoleProcessing(int num, HObject inputImage);
	//H401 ����۴����仨���׵�ƽ�淨ʸ��Բ�� 
	void stepFourLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//H402 ���ǰ�����仨�����ƽ�淨ʸ��Բ�� 
	void stepFourRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//H403 ����۴����仨����λ��

	//H404 ���ǰ�����仨����λ��

	//H501 ��ⷢ���������׵�ƽ�淨ʸ��Բ��
	void stepFiveLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//H502 ���ǰ�����仨�����ƽ�淨ʸ��Բ��
	void stepFiveRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	//H503 ��ⷢ����������λ��

	//H504 ���ǰ�����仨����λ��



	//����
	void v_stepThreeFirstLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	void v_stepThreeFirstRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	void v_stepThreeSecondLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
	void v_stepThreeSecondRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond);
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
	void sendLeftGratingData(vector<double> leftRunDis);
	//�����Ҳ�ɼ�ʱ������ʵ���˶���
	void sendRightGratingData(vector<double> rightRunDis);
	//�����߳�
	void endThread();
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

	//���˶�����
	void motionActionSlot();
	//���������
	void cameraActionSlot();
private:
	VisualProcessing *vp;
	CoreAlgorithm *alg;
	QMutex mutex;
	MotionControl *motion;
	CameraCapture *cap;
	SerialCommunication *serial;
};
