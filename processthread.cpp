#define SETUP
//#define READIMAGE
//#define PROGRESS

#include "processthread.h"

ProcessThread::ProcessThread(QObject *parent)
	: QThread(parent)
{
	threadStatus = false;
	vp = new VisualProcessing;
	alg = new CoreAlgorithm(0);
	//��λ��Ϣ
	connect(this, &ProcessThread::sendStepNum, vp, &VisualProcessing::receiveStepNum);
	connect(vp, &VisualProcessing::sendStepParam, this, &ProcessThread::receiveStepParam);
	connect(this, &ProcessThread::sendProcessingInformation, vp, &VisualProcessing::receiveInformation);
	connect(vp, &VisualProcessing::sendProcessingResult, this, &ProcessThread::receiveProcessingResult);

	qRegisterMetaType<HObject>("HObject");
	qRegisterMetaType<CoreAlgorithm::PlaneNormal>("CoreAlgorithm::PlaneNormal");
	qRegisterMetaType<QString>("QString");//����ͼƬ��������
	qRegisterMetaType<CoreAlgorithm::StereoCircle>("CoreAlgorithm::StereoCircle");
	qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr>("pcl::PointCloud<pcl::PointXYZ>::Ptr");
}

ProcessThread::~ProcessThread()
{
	delete vp;
	vp = nullptr;
	delete alg;
	alg = nullptr;
}

void ProcessThread::run()
{
	mutex.lock();
	while (threadStatus){
		switch (stepNum)
		{
		case 101:
			stepOneFirst();
			break;
		default:
			break;
		}
		
	}
	mutex.unlock();
}

void ProcessThread::stepOneFirst()
{
	//ͳ�Ƴ�������ʱ��
	clock_t start, finish;
	double totaltime;
	start = clock();
	Sleep(100);
	LeftScannerRun(101, leftCircle_cloudPoints, leftCenterResult);
	Sleep(500);
#ifndef SETUP
	motion->CheckRun();
	Sleep(100);
#endif
	sendStatusUpdate(QStringLiteral("���ͷɨ�����"), 0);
	Sleep(100);
#ifndef SETUP
	//ת̨��ת180��.
	motion->platformLtoRRotate();
#endif
	Sleep(500);
	RightScannerRun(102, transmissionCaseCircleFirst_cloudPoints, transmissionCaseCenterResultFirst);
	sendStatusUpdate(QStringLiteral("�Ҳ�ͷɨ�����"), 0);
#ifndef SETUP
	motion->CheckRun();
	Sleep(100);
	motion->platformRtoLRotate();
#endif // !SETUP
	cout << "Բ��: " << endl;
	/*cout << leftCenterResult.center.x << " " << leftCenterResult.center.y << " " << leftCenterResult.center.z << endl;*/
	cout << transmissionCaseCenterResultFirst.center.x
		<< " " << transmissionCaseCenterResultFirst.center.y
		<< " " << transmissionCaseCenterResultFirst.center.z << endl;
	//��ʾ��Ե���Ƽ�Բ��
#ifndef PROGRESS

#endif//��PROGRESS
	sendStatusUpdate(QStringLiteral("��λ1��һ�����������"), 0);
	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	cout << "�˳��������ʱ��Ϊ" << totaltime << "�룡" << endl;

	//�����߳�
	threadStatus = false;

}

void ProcessThread::LeftScannerRun(int num, pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult)
{
	//ͳ�Ƴ�������ʱ��
	clock_t start, finish;
	double totaltime;
	double startPosition;
	double endPosition;
	double position;
	//***��������**************************************************************//

	//��ȡ������ز���
	emit sendStepNum(num);
	//��ȡ���쵱ǰλ��
#ifndef SETUP
	startPosition = motion->GetCurrentPositon();
#endif //SETUP

	//�������ͼ
	HObject inputImageFirst, inputImageSecond;
	//��������
	double speed;
	Vector<Point3f> points;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
	start = clock();
#ifndef SETUP
	//��ɾ�߼�������
	serial->clearGratingData();
	Sleep(100);
	//���õ����˶��ٶ�
	motion->setLeadRailVelocity(CameraNoScanSpeed);
	//�˶�����һ�βɼ�λ��
	motion->setLeadRailMotionDistance(CameraFirstNoScanDistance);
	motion->CheckRun();
	cout << "��ʼ�ɼ�λ�ã�" << endl;
	position = motion->GetCurrentPositon();
	Sleep(500);
	//��ȡ��ɾ�߷��������˶�����
	serial->readGratingData(leftRunDis);
	Sleep(500);
	//��ȡ�����˶��ٶ�
	speed = cap->RateToSpeed(cap->hv_RateValue);
	//���õ����˶��ٶ�
	motion->setLeadRailVelocity(speed);
	//��ȡͼ��
	GrabImageStart(cap->hv_AcqHandle, -1);
	//�������
	cap->sendPLC();
	//�������쿪ʼ�˶�, �ƶ�����Ϊ50mm
	motion->setLeadRailMotionDistance(CameraScanDistance);
	GrabImageAsync(&inputImageFirst, cap->hv_AcqHandle, -1);
#endif //SETUP

#ifndef READIMAGE
	QString ImageFileName;
	switch (num)
	{
	case 101:
		ImageFileName = "edge/LeftFirst" + QString::number(imageNum) + ".tif";
		break;
	case 201:
		ImageFileName = "edge/LeftFirst" + QString::number(imageNum) + ".tif";;
		break;
	case 203:
		ImageFileName = "edge/hole1.tif";
		break;
	case 301:
		ImageFileName = "edge/LeftFirst" + QString::number(imageNum) + ".tif";;
		break;
	case 303:
		ImageFileName = "edge/hole1.tif";
		break;
	case 401:
		ImageFileName = "gear/LeftFirst" + QString::number(imageNum) + ".tif";
		break;
	case 501:
		ImageFileName = "gear/LeftFirst" + QString::number(imageNum) + ".tif";
		break;
	case 1010:
		ImageFileName = "edge/LeftFirst" + QString::number(imageNum) + ".tif";
		break;
	case 2010:
		ImageFileName = "edge/LeftFirst" + QString::number(imageNum) + ".tif";;
		break;
	case 2030:
		ImageFileName = "edge/hole1.tif";
		break;
	case 3010:
		ImageFileName = "gear/LeftFirst" + QString::number(imageNum) + ".tif";
		break;
		break;
	default:
		break;
	}

	cout << ImageFileName.toStdString() << endl << endl;
	QByteArray ba = ImageFileName.toLatin1();
	const char *str = ba.data();
	cout << ImageFileName.toStdString() << endl << endl;
	HTuple  ImageName(str);
	ReadImage(&inputImageFirst, ImageName);
#endif // !READIMAGE
	//��ʾ���ұ���ͼƬ
	emit sendImageAndSave(inputImageFirst, 101, "LeftFirst");
	////��ʾͼƬ
	//displayImage(inputImageFirst, 101);
	////����ͼƬ
	//SaveImage(inputImageFirst, "LeftFirst");

#ifndef SETUP
	if (num == 203 || num == 303 || num == 2030){
		HObject  ho_EmptyRegion;
		GenEmptyRegion(&ho_EmptyRegion);
		RegionToBin(ho_EmptyRegion, &inputImageSecond, 255, 0, 4096, 2000);
	}
	else{
		motion->CheckRun();
		//���õ����˶��ٶ�
		motion->setLeadRailVelocity(CameraNoScanSpeed);
		//�˶����ڶ��βɼ�λ��
		motion->setLeadRailMotionDistance(CameraSecondNoScanDistance);
		motion->CheckRun();
		cout << "�ڶ��βɼ�λ�ã�" << endl;
		position = motion->GetCurrentPositon();
		Sleep(500);
		//��ȡ��ɾ�߷��������˶�����
		serial->readGratingData(leftRunDis);
		Sleep(500);
		//�������쿪ʼ�˶�
		motion->setLeadRailVelocity(speed);
		//�������쿪ʼ�˶�, �ƶ�����Ϊ50mm

		//��ȡͼ��
		GrabImageStart(cap->hv_AcqHandle, -1);
		cap->sendPLC();
		motion->setLeadRailMotionDistance(CameraScanDistance);
		GrabImageAsync(&inputImageSecond, cap->hv_AcqHandle, -1);
	}


#endif //SETUP

#ifndef READIMAGE
	switch (num)
	{
	case 101:
		ImageFileName = "edge/LeftSecond" + QString::number(imageNum) + ".tif";
		break;
	case 201:
		ImageFileName = "edge/LeftSecond" + QString::number(imageNum) + ".tif";
		break;
	case 301:
		ImageFileName = "edge/LeftSecond" + QString::number(imageNum) + ".tif";
		break;
	case 401:
		ImageFileName = "gear/LeftSecond" + QString::number(imageNum) + ".tif";
		break;
	case 501:
		ImageFileName = "gear/LeftSecond" + QString::number(imageNum) + ".tif";
		break;
	case 1010:
		ImageFileName = "edge/LeftSecond" + QString::number(imageNum) + ".tif";
		break;
	case 2010:
		ImageFileName = "edge/LeftSecond" + QString::number(imageNum) + ".tif";
		break;
	case 3010:
		ImageFileName = "gear/LeftSecond" + QString::number(imageNum) + ".tif";
		break;
	default:
		break;
	}
	if (num != 203 && num != 303 && num != 2030){
		ba = ImageFileName.toLatin1();
		const char *str1 = ba.data();
		HTuple  ImageName1(str1);
		ReadImage(&inputImageSecond, ImageName1);
	}
	else{
		HObject  ho_EmptyRegion;
		GenEmptyRegion(&ho_EmptyRegion);
		RegionToBin(ho_EmptyRegion, &inputImageSecond, 255, 0, 4096, 2000);
	}

#endif //READIMAGE
	emit sendImageAndSave(inputImageSecond, 102, "LeftSecond");
	////��ʾͼƬ
	//displayImage(inputImageSecond, 102);
	////����ͼƬ
	//SaveImage(inputImageSecond, "LeftSecond");

#ifndef SETUP
	motion->CheckRun();
	Sleep(100);
	cout << "����" << endl;
	cout << "�˶�����λ�ã�" << endl;
	position = motion->GetCurrentPositon();
	Sleep(500);
	//��ȡ�ɼ�����λ��
	endPosition = motion->GetCurrentPositon();
	//���츴λ
	motion->reset(startPosition, endPosition);
	cout << "����" << endl;
	//��������
	emit sendLeftGratingData(leftRunDis);
	//������գ�Ϊ�´δ�����׼��
	leftRunDis.clear();
#endif //SETUP

#ifndef PROGRESS
	switch (num)
	{
	case 101:
		//emit sendProcessingInformation(101, inputImageFirst, inputImageSecond);
		stepOneFirstLeftProcessing(101,  inputImageFirst,  inputImageSecond);
		break;
	case 201:
		emit sendProcessingInformation(201, inputImageFirst, inputImageSecond);
		break;
	case 203:
		emit sendProcessingInformation(203, inputImageFirst, inputImageSecond);
		break;
	case 301:
		emit sendProcessingInformation(301, inputImageFirst, inputImageSecond);
		break;
	case 303:
		emit sendProcessingInformation(303, inputImageFirst, inputImageSecond);
		break;
	case 401:
		emit sendProcessingInformation(401, inputImageFirst, inputImageSecond);
		break;
	case 501:
		emit sendProcessingInformation(501, inputImageFirst, inputImageSecond);
		break;
	case 1010:
		emit sendProcessingInformation(1010, inputImageFirst, inputImageSecond);
		break;
	case 2010:
		emit sendProcessingInformation(2010, inputImageFirst, inputImageSecond);
		break;
	case 2030:
		emit sendProcessingInformation(2030, inputImageFirst, inputImageSecond);
		break;
	case 3010:
		emit sendProcessingInformation(3010, inputImageFirst, inputImageSecond);
		break;
	case 3030:
		emit sendProcessingInformation(3030, inputImageFirst, inputImageSecond);
		break;
	default:
		break;
	}
#endif //PROGRESS
	cout << "over!" << endl;
	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	//��ձ���
	points.clear();

	cout << "�˳��������ʱ��Ϊ" << totaltime << "�룡" << endl;
}

void ProcessThread::RightScannerRun(int num, pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult)
{
	//ͳ�Ƴ�������ʱ��
	clock_t start, finish;
	double totaltime;
	double startPosition;
	double endPosition;
	double position;
	//***��������**************************************************************//
	//��ȡ���쵱ǰλ��
#ifndef SETUP
	startPosition = motion->GetCurrentPositon();
#endif //SETUP
	//��ȡ������ز���
	emit sendStepNum(num);
	//�������ͼ
	HObject inputImageFirst, inputImageSecond;
	//��������
	double speed;
	Vector<Point3f> points;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
	start = clock();
#ifndef SETUP
	serial->clearGratingData();
	Sleep(100);
	motion->setLeadRailVelocity(CameraNoScanSpeed);
	//�˶�����һ�βɼ�λ��
	motion->setLeadRailMotionDistance(CameraFirstNoScanDistance);
	motion->CheckRun();
	cout << "��ʼ�ɼ�λ�ã�" << endl;
	position = motion->GetCurrentPositon();
	Sleep(500);
	//��ȡ��ɾ�߷��������˶�����
	serial->readGratingData(rightRunDis);
	Sleep(500);
	//��ȡ�����˶��ٶ�
	speed = cap->RateToSpeed(cap->hv_RateValue);
	//���õ����˶��ٶ�
	motion->setLeadRailVelocity(speed);
	//��ȡͼ��
	GrabImageStart(cap->hv_AcqHandle, -1);
	cap->sendPLC();
	//�������쿪ʼ�˶�, �ƶ�����Ϊ50mmCameraScanDistance
	motion->setLeadRailMotionDistance(CameraScanDistance);
	GrabImageAsync(&inputImageFirst, cap->hv_AcqHandle, -1);
#endif //SETUP

#ifndef READIMAGE
	QString ImageFileName;
	switch (num)
	{
	case 102:
		ImageFileName = "edge/LeftFirst" + QString::number(imageNum) + ".tif";
		break;
	case 104:
		ImageFileName = "edge/LeftFirst2.tif";
		break;
	case 202:
		ImageFileName = "edge/LeftFirst" + QString::number(imageNum) + ".tif";
		break;
	case 204:
		ImageFileName = "edge/hole1.tif";
		break;
	case 302:
		ImageFileName = "edge/LeftFirst" + QString::number(imageNum) + ".tif";
		break;
	case 304:
		ImageFileName = "edge/hole1.tif";
		break;
	case 402:
		ImageFileName = "gear/RightFirst" + QString::number(imageNum) + ".tif";
		break;
	case 502:
		ImageFileName = "gear/RightFirst" + QString::number(imageNum) + ".tif";
		break;
	case 1020:
		ImageFileName = "edge/LeftFirst" + QString::number(imageNum) + ".tif";
		break;
	case 1040:
		ImageFileName = "edge/LeftFirst2.tif";
		break;
	case 2020:
		ImageFileName = "edge/LeftFirst" + QString::number(imageNum) + ".tif";
		break;
	case 2040:
		ImageFileName = "edge/hole1.tif";
		break;
	case 3020:
		ImageFileName = "gear/RightFirst" + QString::number(imageNum) + ".tif";
		break;
	default:
		break;
	}
	cout << ImageFileName.toStdString() << endl << endl;
	QByteArray ba = ImageFileName.toLatin1();
	const char *str = ba.data();
	cout << ImageFileName.toStdString() << endl << endl;
	HTuple  ImageName(str);
	ReadImage(&inputImageFirst, ImageName);
#endif // !READIMAGE
	emit sendImageAndSave(inputImageFirst, 201, "RightFirst");
	////��ʾͼƬ
	//displayImage(inputImageFirst, 201);
	////����ͼƬ
	//SaveImage(inputImageFirst, "RightFirst");

#ifndef SETUP
	if (num == 204 || num == 304 || num == 2040){
		HObject  ho_EmptyRegion;
		GenEmptyRegion(&ho_EmptyRegion);
		RegionToBin(ho_EmptyRegion, &inputImageSecond, 255, 0, 4096, 2000);
	}
	else{
		motion->CheckRun();
		Sleep(100);
		//���õ����˶��ٶ�
		motion->setLeadRailVelocity(CameraNoScanSpeed);
		//�˶����ڶ��βɼ�λ��
		motion->setLeadRailMotionDistance(CameraSecondNoScanDistance);
		motion->CheckRun();
		cout << "�ڶ��βɼ�λ�ã�" << endl;
		position = motion->GetCurrentPositon();
		Sleep(500);
		//��ȡ��ɾ�߷��������˶�����
		serial->readGratingData(rightRunDis);
		Sleep(500);
		//�������쿪ʼ�˶�
		motion->setLeadRailVelocity(speed);
		//�������쿪ʼ�˶�, �ƶ�����Ϊ50mm

		//��ȡͼ��
		GrabImageStart(cap->hv_AcqHandle, -1);
		cap->sendPLC();
		motion->setLeadRailMotionDistance(CameraScanDistance);
		GrabImageAsync(&inputImageSecond, cap->hv_AcqHandle, -1);
	}


#endif //SETUP

#ifndef READIMAGE
	switch (num)
	{
	case 102:
		ImageFileName = "edge/LeftSecond" + QString::number(imageNum) + ".tif";
		break;
	case 104:
		ImageFileName = "edge/LeftSecond2.tif";
		break;
	case 202:
		ImageFileName = "edge/LeftSecond" + QString::number(imageNum) + ".tif";
		break;
		/*case 204:
		ImageFileName = "edge/LeftSecond" + ui.numEdit->text() + ".tif";
		break;*/
	case 302:
		ImageFileName = "edge/LeftSecond" + QString::number(imageNum) + ".tif";
		break;
	case 402:
		ImageFileName = "gear/RightSecond" + QString::number(imageNum) + ".tif";
		break;
	case 502:
		ImageFileName = "gear/RightSecond" + QString::number(imageNum) + ".tif";
		break;
	case 1020:
		ImageFileName = "edge/LeftSecond" + QString::number(imageNum) + ".tif";
		break;
	case 1040:
		ImageFileName = "edge/LeftSecond2.tif";
		break;
	case 2020:
		ImageFileName = "edge/LeftSecond" + QString::number(imageNum) + ".tif";
		break;
	case 3020:
		ImageFileName = "gear/RightSecond" + QString::number(imageNum) + ".tif";
		break;
	default:
		break;
	}
	if (num != 204 && num != 304 && num != 2040){
		ba = ImageFileName.toLatin1();
		const char *str1 = ba.data();
		HTuple  ImageName1(str1);
		ReadImage(&inputImageSecond, ImageName1);
	}
	else{
		HObject  ho_EmptyRegion;
		GenEmptyRegion(&ho_EmptyRegion);
		RegionToBin(ho_EmptyRegion, &inputImageSecond, 255, 0, 4096, 2000);
	}


#endif //READIMAGE
	emit sendImageAndSave(inputImageSecond, 202, "RightSecond");

	////��ʾͼƬ
	//displayImage(inputImageSecond, 202);
	////����ͼƬ
	//SaveImage(inputImageSecond, "RightSecond");
#ifndef SETUP
	motion->CheckRun();
	Sleep(100);
	cout << "����" << endl;
	cout << "�˶�����λ�ã�" << endl;
	position = motion->GetCurrentPositon();
	Sleep(500);
	//��ȡ�ɼ�����λ��
	endPosition = motion->GetCurrentPositon();
	//���츴λ
	motion->reset(startPosition, endPosition);
	cout << "����" << endl;
	//��������
	emit sendRightGratingData(rightRunDis);
	//������գ�Ϊ�´δ�����׼��
	rightRunDis.clear();
#endif //SETUP

#ifndef PROGRESS 
	switch (num)
	{
	case 102:
		stepOneSecondRightProcessing(102,inputImageFirst,inputImageSecond);
		break;
	case 104:
		emit sendProcessingInformation(104, inputImageFirst, inputImageSecond);
		break;
	case 202:
		emit sendProcessingInformation(202, inputImageFirst, inputImageSecond);
		break;
	case 204:
		emit sendProcessingInformation(204, inputImageFirst, inputImageSecond);
		break;
	case 302:
		emit sendProcessingInformation(302, inputImageFirst, inputImageSecond);
		break;
	case 304:
		emit sendProcessingInformation(304, inputImageFirst, inputImageSecond);
		break;
	case 402:
		emit sendProcessingInformation(402, inputImageFirst, inputImageSecond);
		break;
	case 502:
		emit sendProcessingInformation(502, inputImageFirst, inputImageSecond);
		break;
	case 1020:
		emit sendProcessingInformation(1020, inputImageFirst, inputImageSecond);
		break;
	case 1040:
		emit sendProcessingInformation(1040, inputImageFirst, inputImageSecond);
		break;
	case 2020:
		emit sendProcessingInformation(2020, inputImageFirst, inputImageSecond);
		break;
	case 2040:
		emit sendProcessingInformation(2040, inputImageFirst, inputImageSecond);
		break;
	case 3020:
		emit sendProcessingInformation(3020, inputImageFirst, inputImageSecond);
		break;
	case 3040:
		emit sendProcessingInformation(3040, inputImageFirst, inputImageSecond);
		break;
	default:
		break;
	}

#endif //PROGRESS
	cout << "over!" << endl;
	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	//��ձ���
	points.clear();

	cout << "�˳��������ʱ��Ϊ" << totaltime << "�룡" << endl;
}

void ProcessThread::receiveStepParam(double CamNoScanSpeed, double CamScanDistance, double CamFirstNoScanDistance, double CamSecondNoScanDistance)
{
	if (CamNoScanSpeed == -1 && CamScanDistance == -1){
		cout << "param error" << endl;
		CameraNoScanSpeed = 20;
		CameraScanDistance = 50;
		CameraFirstNoScanDistance = 20;
		CameraSecondNoScanDistance = 30;
	}
	else{
		CameraNoScanSpeed = CamNoScanSpeed;
		CameraScanDistance = CamScanDistance;
		CameraFirstNoScanDistance = CamFirstNoScanDistance;
		CameraSecondNoScanDistance = CamSecondNoScanDistance;
	}
	cout << "������Ϣ��" << endl;
	cout << "	��ͷ��ɨ��ʱ�˶��ٶ�" << CameraNoScanSpeed << endl;
	cout << "	��ͷɨ��ʱ�˶�����" << CameraScanDistance << endl;
	cout << "	��ͷ�˶�����һ����λ�õ��˶�����" << CameraFirstNoScanDistance << endl;
	cout << "	��ͷ�˶����ڶ�����λ�õ��˶�����" << CameraSecondNoScanDistance << endl;
}

void ProcessThread::receiveImageNum(int num)
{
	imageNum = num;
}

void ProcessThread::receiveProcessingResult(bool planeparamStatus/*ƽ�淨ʸ����״̬*/, CoreAlgorithm::PlaneNormal planeparam/*ƽ�淨ʸ*/, CoreAlgorithm::StereoCircle centerResult/*�ռ�Բ��*/, pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr/*��Ե���� PCL��ʽ*/, int num/*������*/)
{
	emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
}

void ProcessThread::stepOneFirstLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond)
{
	//�����ͷ����ϵ��Y������
	double LeftIncrementFirst;
	double LeftIncrementSecond;
	/*LeftIncrementFirst = ui.LeftRunFirst_StepOne->text().toDouble();
	LeftIncrementSecond = ui.LeftRunFirst_StepOne->text().toDouble() + ui.LeftCameraScan_StepOne->text().toDouble()
	+ ui.LeftRunSecond_StepOne->text().toDouble();*/
	vp->calculateMoveDistance(num,LeftIncrementFirst,LeftIncrementSecond);
#ifndef SETUP
	LeftIncrementFirst = leftRunDis[0];
	LeftIncrementSecond = leftRunDis[1];
#endif//SETUP
	QString runtime = vp->ReturnTime();
	QString runfileName = vp->engineNumFileName + "/outputFile/StepOne/GratingData.csv";//д���ļ���Ŀ¼
	QFile runfile(runfileName);
	runfile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
	QTextStream runout(&runfile);
	runout << runtime << "," << "left" << "," << LeftIncrementFirst << "," << LeftIncrementSecond << "\n";
	runfile.close();

	//����ƽ�洦��ͼƬ
	HObject planeFirstImage, planeSecondImage;
	//������Ե����ͼƬ
	//HObject circleFirstImage, circleSecondImage;
	//����ƽ��㼯
	Vector<Point3f> planePointsFirst, planePointsSecond;
	//����ͳһ����ϵ��ƽ��㼯
	Vector<Point3f> unifyPlanePoints;
	//������Ե�㼯
	//Vector<Point3f> circlePointsFirst, circlePointsSecond;
	//����ͳһ����ϵ�±�Ե�㼯
	//Vector<Point3f> unifyCirclePoints;
	//����ͼ������ true or false
	bool status_planeFirst, status_planeSecond;
	//bool status_circleFirst, status_circleSecond;
	//��ȡƽ�洦��ͼƬ
	planeFirstImage = alg->getPlaneImage(inputImageFirst, status_planeFirst);
	planeSecondImage = alg->getPlaneImage(inputImageSecond, status_planeSecond);
	//��ȡ��Ե����ͼƬ
	//circleFirstImage = alg->getFlangeEdgeImage(inputImageFirst, status_circleFirst);
	//circleSecondImage = alg->getFlangeEdgeImage(inputImageSecond, status_circleSecond);
	//��ȡƽ��㼯
	planePointsFirst = alg->depthMapCalibration(planeFirstImage, status_planeFirst);
	planePointsSecond = alg->depthMapCalibration(planeSecondImage, status_planeSecond);
	cout << "1ƽ��㼯������" << planePointsFirst.size() << endl;
	cout << "2ƽ��㼯������" << planePointsSecond.size() << endl;
	//��ȡ��Ե�㼯
	//circlePointsFirst = alg->depthMapCalibration(circleFirstImage, status_circleFirst);
	//circlePointsSecond = alg->depthMapCalibration(circleSecondImage, status_circleSecond);
	//cout << "1��Ե�㼯������" << circlePointsFirst.size() << endl;
	//cout << "2��Ե�㼯������" << circlePointsSecond.size() << endl;
	//�ɼ��㣬ͳһ����ϵ
	for (int i = 0; i < planePointsFirst.size(); i++){
		unifyPlanePoints.push_back(Point3f(planePointsFirst[i].x, planePointsFirst[i].y + LeftIncrementFirst, planePointsFirst[i].z));
	}

	for (int i = 0; i < planePointsSecond.size(); i++){
		unifyPlanePoints.push_back(Point3f(planePointsSecond[i].x, planePointsSecond[i].y + LeftIncrementSecond, planePointsSecond[i].z));
	}
	if (unifyPlanePoints.size() != 0)
	{
		//����ƽ����ƣ�PCL��
		pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
		//������Ե���ƣ�PCL��
		//pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		//ƽ��㼯ת��ΪPCL����
		alg->threeDimensionToPCLPoints(unifyPlanePoints, plane_cloudPoints);
		//�����ռ�ƽ�淨ʸ���ݽṹ
		CoreAlgorithm::PlaneNormal planeparam;
		//��ȡƽ�淨ʸ
		planeparam = alg->getPlaneNormal(plane_cloudPoints);
		

		//����任
		//filterBorderPoints = MeasureToRobot(filterBorderPoints);
		//	centerResult = MeasureToRobot(centerResult);
		planeparam = vp->MeasureToRobotTool(planeparam);
		planeparam = vp->RobotToolToBase(planeparam);
		cout << "����任��" << endl;
		//cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
		cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;

		QString time = vp->ReturnTime();
		QString fileName = vp->engineNumFileName + "/outputFile/StepOne/firstPart/engineParam.csv";//д���ļ���Ŀ¼
		QFile file(fileName);
		file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
		QTextStream out(&file);
		out << time << "," << num << "," << planeparam.A << "," << planeparam.B << "," << planeparam.C << "\n";
		file.close();
		bool planeparamStatus = true;
		//�����ռ�����
		CoreAlgorithm::StereoCircle centerResult;
		//������Ե���ƣ�PCL��
		pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);

	}
	else
	{
		bool planeparamStatus = false;
		//�����ռ�ƽ�淨ʸ���ݽṹ
		CoreAlgorithm::PlaneNormal planeparam;
		QString time = vp->ReturnTime();
		QString fileName = vp->engineNumFileName + "/outputFile/StepOne/firstPart/engineNormal.csv";//д���ļ���Ŀ¼
		QFile file(fileName);
		file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
		QTextStream out(&file);
		out << time << "," << num << "," << "null" << "," << "null" << "," << "null" << "\n";
		file.close();
		//�����ռ�����
		CoreAlgorithm::StereoCircle centerResult;
		//������Ե���ƣ�PCL��
		pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
	}
}

void ProcessThread::stepOneSecondRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond)
{
	//�����ͷ����ϵ��Y������
	double RightIncrementFirst;
	double RightIncrementSecond;
	vp->calculateMoveDistance(num, RightIncrementFirst, RightIncrementSecond);
	/*RightIncrementFirst = ui.RightRunFirst_StepOne->text().toDouble();
	RightIncrementSecond = ui.RightRunFirst_StepOne->text().toDouble() + ui.RightCameraScan_StepOne->text().toDouble()
	+ ui.RightRunSecond_StepOne->text().toDouble();*/
#ifndef SETUP
	RightIncrementFirst = rightRunDis[0];
	RightIncrementSecond = rightRunDis[1];
#endif//SETUP
	QString runtime = vp->ReturnTime();
	QString runfileName = vp->engineNumFileName + "/outputFile/stepOne/GratingData.csv";//д���ļ���Ŀ¼
	QFile runfile(runfileName);
	runfile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
	QTextStream runout(&runfile);
	runout << runtime << "," << "right" << "," << RightIncrementFirst << "," << RightIncrementSecond << "\n";
	runfile.close();
	//����ƽ�洦��ͼƬ
	HObject planeFirstImage, planeSecondImage;
	//������Ե����ͼƬ
	HObject circleFirstImage, circleSecondImage;
	//����ƽ��㼯
	Vector<Point3f> planePointsFirst, planePointsSecond;
	//����ͳһ����ϵ��ƽ��㼯
	Vector<Point3f> unifyPlanePoints;
	//������Ե�㼯
	Vector<Point3f> circlePointsFirst, circlePointsSecond;
	//����ͳһ����ϵ�±�Ե�㼯
	Vector<Point3f> unifyCirclePoints;
	//����ͼ������ true or false
	bool status_planeFirst, status_planeSecond;
	bool status_circleFirst, status_circleSecond;
	//��ȡƽ�洦��ͼƬ
	planeFirstImage = alg->getPlaneImage(inputImageFirst, status_planeFirst);
	planeSecondImage = alg->getPlaneImage(inputImageSecond, status_planeSecond);
	//��ȡ��Ե����ͼƬ
	if (num == 102 || num == 104){
		circleFirstImage = alg->getFlangeEdgeImage(inputImageFirst, status_circleFirst);
		circleSecondImage = alg->getFlangeEdgeImage(inputImageSecond, status_circleSecond);
	}//���÷�����
	if (num == 1020 || num == 1040){
		circleFirstImage = alg->getFlangeEdgeImage(inputImageFirst, status_circleFirst);
		circleSecondImage = alg->getFlangeEdgeImage(inputImageSecond, status_circleSecond);
	}//���÷�����

	//��ȡƽ��㼯
	planePointsFirst = alg->depthMapCalibration(planeFirstImage, status_planeFirst);
	planePointsSecond = alg->depthMapCalibration(planeSecondImage, status_planeSecond);
	cout << "1ƽ��㼯������" << planePointsFirst.size() << endl;
	cout << "2ƽ��㼯������" << planePointsSecond.size() << endl;
	//��ȡ��Ե�㼯
	circlePointsFirst = alg->depthMapCalibration(circleFirstImage, status_circleFirst);
	circlePointsSecond = alg->depthMapCalibration(circleSecondImage, status_circleSecond);
	cout << "1��Ե�㼯������" << circlePointsFirst.size() << endl;
	cout << "2��Ե�㼯������" << circlePointsSecond.size() << endl;
	//�ɼ��㣬ͳһ����ϵ
	for (int i = 0; i < planePointsFirst.size(); i++){
		unifyPlanePoints.push_back(Point3f(planePointsFirst[i].x, planePointsFirst[i].y + RightIncrementFirst, planePointsFirst[i].z));
	}

	for (int i = 0; i < planePointsSecond.size(); i++){
		unifyPlanePoints.push_back(Point3f(planePointsSecond[i].x, planePointsSecond[i].y + RightIncrementSecond, planePointsSecond[i].z));
	}

	for (int i = 0; i < circlePointsFirst.size(); i++){
		unifyCirclePoints.push_back(Point3f(circlePointsFirst[i].x, circlePointsFirst[i].y + RightIncrementFirst, circlePointsFirst[i].z));
	}

	for (int i = 0; i < circlePointsSecond.size(); i++){
		unifyCirclePoints.push_back(Point3f(circlePointsSecond[i].x, circlePointsSecond[i].y + RightIncrementSecond, circlePointsSecond[i].z));
	}
	cout << "ͳһ�����ƽ��㼯������" << unifyPlanePoints.size() << endl;
	cout << "ͳһ������Ե�㼯������" << unifyCirclePoints.size() << endl;

	QString time = vp->ReturnTime();
	QString fileName;
	if (num == 102)
	{
		fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/frontDriveBoxUnifyCirclePoints/" + time + " frontDriveBoxUnifyCirclePoints.txt";
	}
	if (num == 1020){
		fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/syntheticalDriveBoxUnifyCirclePoints/" + time + " syntheticalDriveBoxUnifyCirclePoints.txt";
	}
	if (num == 104)
	{
		fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/frontDriveBoxUnifyCirclePoints/" + time + " frontDriveBoxUnifyCirclePoints.txt";
	}
	if (num == 1040){
		fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/syntheticalDriveBoxUnifyCirclePoints/" + time + " syntheticalDriveBoxUnifyCirclePoints.txt";
	}
	QFile file(fileName);
	file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
	QTextStream out(&file);
	for (int i = 0; i < unifyCirclePoints.size(); i++){
		out << unifyCirclePoints[i].x << " " << unifyCirclePoints[i].y << " " << unifyCirclePoints[i].z << endl;
	}
	file.close();
	if (unifyPlanePoints.size() != 0)//�ȼ���Ƿ���Խ���ƽ�����
	{
		//����ƽ����ƣ�PCL��
		pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
		//������Ե���ƣ�PCL��
		pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		//ƽ��㼯ת��ΪPCL����
		alg->threeDimensionToPCLPoints(unifyPlanePoints, plane_cloudPoints);
		//�����ռ�ƽ�淨ʸ���ݽṹ
		CoreAlgorithm::PlaneNormal planeparam;
		//��ȡƽ�淨ʸ
		planeparam = alg->getPlaneNormal(plane_cloudPoints);
		if (unifyCirclePoints.size() != 0)//����Ƿ���Խ���Բ�����
		{
			//�����Ե�㵽��Ͽռ�ƽ��ľ��룬�����þ�����ֵ��ȡ������ֵ�ĵ�
			//�������������
			Vector<Point3f> out_circlePoints;
			//�����㵽ƽ����뼯��
			Vector<double> distance;
			//��������ֵΪmm����������ֵΪmm
			alg->setDistanceThresholdAndGetPoints(unifyCirclePoints, planeparam, distance, 0, 0.7, out_circlePoints);
			cout << "�������������" << out_circlePoints.size() << endl;
			ofstream outfile;
			if (num == 102)
			{
				fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/frontDriveBoxCirclePointsThroughDistanceThreshold/" + time
					+ " frontDriveBoxCirclePointsThroughDistanceThreshold.txt";
			}
			if (num == 1020){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/syntheticalDriveBoxCirclePointsThroughDistanceThreshold/" + time
					+ " syntheticalDriveBoxCirclePointsThroughDistanceThreshold.txt";
			}
			if (num == 104){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/frontDriveBoxCirclePointsThroughDistanceThreshold/" + time
					+ " frontDriveBoxCirclePointsThroughDistanceThreshold.txt";
			}
			if (num == 1040){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/syntheticalDriveBoxCirclePointsThroughDistanceThreshold/" + time
					+ " syntheticalDriveBoxCirclePointsThroughDistanceThreshold.txt";
			}

			outfile.open(fileName.toStdString());
			for (int i = 0; i < out_circlePoints.size(); i++){
				outfile << out_circlePoints[i].x << " " << out_circlePoints[i].y << " " << out_circlePoints[i].z << endl;
			}
			outfile.close();

			//�����ռ�����

			CoreAlgorithm::StereoCircle centerResult;
			//���пռ�Բ���
			//�뾶Լ��
			double R;
			vp->getRestrictRadius(num, R);
			Vector<Point3f> subpoints;
			//�����Ե�㵽���ƽ���ͶӰ��
			subpoints = alg->ComputedEdgeProjectionPoints(out_circlePoints, planeparam);
			centerResult = alg->fitSteroCircle(subpoints, planeparam, R);
			double averageDistance;
			averageDistance = alg->CalculateBorderPointsToCenterAverageDistance(subpoints, centerResult);
			cout << endl << "��Ե�㵽�ռ�Բ�ĵ�ƽ������Ϊ�� " << averageDistance << endl << endl;
			cout << "�ռ�Բ�����꣺" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			Vector<double>distances;
			alg->CalculateBorderPointsToCenterDistance(subpoints, centerResult, distances);
			Vector<Point3f>filterBorderPoints;
			filterBorderPoints = alg->filterBorderPointsOnDistanceThrehold(subpoints, distances, R, 0.05);
			cout << "����ǰ���� = " << subpoints.size() << endl;
			cout << "���˺���� = " << filterBorderPoints.size() << endl;
			centerResult = alg->fitSteroCircle(filterBorderPoints, planeparam, R);
			cout << "�ռ�Բ�����꣺" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			averageDistance = alg->CalculateBorderPointsToCenterAverageDistance(filterBorderPoints, centerResult);
			cout << endl << "��Ե�㵽�ռ�Բ�ĵ�ƽ������Ϊ�� " << averageDistance << endl << endl;
			if (num == 102 || num == 1020){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/frontDriveBoxFilterBorderPoints/" + time
					+ " frontDriveBoxFilterBorderPoints.txt";
			}
			if (num == 1020){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/syntheticalDriveBoxFilterBorderPoints/" + time
					+ " syntheticalDriveBoxFilterBorderPoints.txt";
			}
			if (num == 104 || num == 1040){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/frontDriveBoxFilterBorderPoints/" + time
					+ " frontDriveBoxFilterBorderPoints.txt";
			}
			if (num == 1040){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/syntheticalDriveBoxFilterBorderPoints/" + time
					+ " syntheticalDriveBoxFilterBorderPoints.txt";
			}

			outfile.open(fileName.toStdString());
			for (int i = 0; i < filterBorderPoints.size(); i++){
				outfile << filterBorderPoints[i].x << " " << filterBorderPoints[i].y << " " << filterBorderPoints[i].z << endl;
			}
			outfile.close();

			//����任
			//�Ҳ�ͷ����ϵ�����ͷ����ϵ
			filterBorderPoints = vp->RightToLeft(filterBorderPoints);
			centerResult = vp->RightToLeft(centerResult);
			planeparam = vp->RightToLeft(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//��ͷ����ϵ�������˹�������ϵ
			filterBorderPoints = vp->MeasureToRobotTool(filterBorderPoints);
			centerResult = vp->MeasureToRobotTool(centerResult);
			planeparam = vp->MeasureToRobotTool(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//�����˹�������ϵ�������˻�������ϵ
			filterBorderPoints = vp->RobotToolToBase(filterBorderPoints);
			centerResult = vp->RobotToolToBase(centerResult);
			planeparam = vp->RobotToolToBase(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//���洦����
			time = vp->ReturnTime();
			if (num == 102){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/frontDriveBoxParam.csv";//д���ļ���Ŀ¼
			}
			if (num == 1020){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/syntheticalDriveBoxParam.csv";//д���ļ���Ŀ¼
			}
			if (num == 104){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/frontDriveBoxParam.csv";//д���ļ���Ŀ¼
			}
			if (num == 1040){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/syntheticalDriveBoxParam.csv";//д���ļ���Ŀ¼
			}
			QString ccircleStatusStr;
			if (abs(averageDistance - R) < 0.1){
				centerResult.status = true;
				ccircleStatusStr = "true";
			}
			else{
				centerResult.status = false;
				ccircleStatusStr = "false";
			}
			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time << "," << num << "," << centerResult.center.x << "," << centerResult.center.y << "," << centerResult.center.z << "," << ccircleStatusStr << ","
				<< planeparam.A << "," << planeparam.B << "," << planeparam.C << ","
				<< averageDistance << "\n";
			file.close();
			//�����������
			if (num == 102){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/frontDriveBoxFinalBorderPoints/" + time
					+ " frontDriveBoxFinalBorderPoints.txt";
			}
			if (num == 1020){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/syntheticalDriveBoxFinalBorderPoints/" + time
					+ " syntheticalDriveBoxFinalBorderPoints.txt";
			}
			if (num == 104){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/frontDriveBoxFinalBorderPoints/" + time
					+ " frontDriveBoxFinalBorderPoints.txt";
			}
			if (num == 1040){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/syntheticalDriveBoxFinalBorderPoints/" + time
					+ " syntheticalDriveBoxFinalBorderPoints.txt";
			}
			outfile.open(fileName.toStdString());
			for (int i = 0; i < filterBorderPoints.size(); i++){
				outfile << filterBorderPoints[i].x << " " << filterBorderPoints[i].y << " " << filterBorderPoints[i].z << endl;
			}
			outfile.close();


			bool planeparamStatus = true;
			//��Ե�㼯ת��ΪPCL����
			alg->threeDimensionToPCLPoints(filterBorderPoints, borderCloud_ptr);
			emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
		}
		else{
			CoreAlgorithm::StereoCircle centerResult;
			centerResult.status = false;
			bool planeparamStatus = true;
			if (num == 102){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/frontDriveBoxParam.csv";//д���ļ���Ŀ¼
			}
			if (num == 1020){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/syntheticalDriveBoxParam.csv";//д���ļ���Ŀ¼
			}
			if (num == 104){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/frontDriveBoxParam.csv";//д���ļ���Ŀ¼
			}
			if (num == 1040){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/syntheticalDriveBoxParam.csv";//д���ļ���Ŀ¼
			}
			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time << "," << num << "," << "null" << "," << "null" << "," << "null" << "," << "false" << ","
				<< planeparam.A << "," << planeparam.B << "," << planeparam.C << "," << "null" << "\n";
			file.close();
			emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
		}


	}
	else
	{
		bool planeparamStatus = false;
		//�����ռ�ƽ�淨ʸ���ݽṹ
		CoreAlgorithm::PlaneNormal planeparam;
		//�����ռ�����
		CoreAlgorithm::StereoCircle centerResult;
		centerResult.status = false;
		//������Ե���ƣ�PCL��
		pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		time = vp->ReturnTime();
		if (num == 102){
			fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/frontDriveBoxParam.csv";//д���ļ���Ŀ¼
		}
		if (num == 1020){
			fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/syntheticalDriveBoxParam.csv";//д���ļ���Ŀ¼
		}
		if (num == 104){
			fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/frontDriveBoxParam.csv";//д���ļ���Ŀ¼
		}
		if (num == 1040){
			fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/syntheticalDriveBoxParam.csv";//д���ļ���Ŀ¼
		}
		QFile file(fileName);
		file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
		QTextStream out(&file);
		out << time << "," << num << "," << "null" /*Բ��X*/ << "," << "null" /*Բ��Y*/ << "," << "null"/*Բ��Z*/ << "," << "false"/*Բ�����״̬*/ << ","
			<< "null"/*��ʸA*/ << "," << "null"/*��ʸB*/ << "," << "null"/*��ʸC*/ << "," << "null" /*��Ե�㵽Բ�ĵ�ƽ������*/ << "\n";
		file.close();
		emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
	}
}