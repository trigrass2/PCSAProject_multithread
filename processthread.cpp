#define SETUP
//#define READIMAGE
//#define PROGRESS

#include "processthread.h"

ProcessThread::ProcessThread()
	: QThread()
{
	threadStatus = false;
	vp = new VisualProcessing;
	alg = new CoreAlgorithm(0);
	motion = new MotionControl;
	cap = new CameraCapture;
	serial = new SerialCommunication;
	//��λ��Ϣ
	connect(this, &ProcessThread::sendStepNum, vp, &VisualProcessing::receiveStepNum);
	connect(vp, &VisualProcessing::sendStepParam, this, &ProcessThread::receiveStepParam);
	connect(this, &ProcessThread::sendProcessingInformation, vp, &VisualProcessing::receiveInformation);
	connect(vp, &VisualProcessing::sendProcessingResult, this, &ProcessThread::receiveProcessingResult);

	connect(cap, &CameraCapture::toPLCInfo, serial, &SerialCommunication::sendPlC);
	connect(motion, &MotionControl::toPLCInfo, serial, &SerialCommunication::sendPlC);

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
	delete cap;
	cap = nullptr;
	delete serial;
	serial = nullptr;
}

void ProcessThread::run()
{
	mutex.lock();
	step();
	mutex.unlock();
}

void ProcessThread::step()
{
	//ͳ�Ƴ�������ʱ��
	clock_t start, finish;
	double totaltime;
	start = clock();
	//��ȡ��λ�ɼ����
	//stepCapture *scNum;
	//scNum = getStepCapture(stepNum);
	if (stepNum % 2 == 1&&stepNum!=103){
#ifndef SETUP
		motion->CheckRun();
		Sleep(100);
#endif
		LeftScannerRun(stepNum, leftCircle_cloudPoints, leftCenterResult);
		Sleep(500);
#ifndef SETUP
		motion->CheckRun();
		Sleep(100);
#endif
		sendStatusUpdate(QStringLiteral("���ͷɨ�����"), 0);
		Sleep(100);
	}

	if (stepNum % 2 == 0 || stepNum == 103){
#ifndef SETUP
		motion->CheckRun();
		Sleep(100);
#endif
#ifndef SETUP
		//ת̨��ת180��.
		motion->platformLtoRRotate();
#endif
		Sleep(500);
		RightScannerRun(stepNum, transmissionCaseCircleFirst_cloudPoints, transmissionCaseCenterResultFirst);
		sendStatusUpdate(QStringLiteral("�Ҳ�ͷɨ�����"), 0);
#ifndef SETUP
		motion->CheckRun();
		Sleep(100);
		motion->platformRtoLRotate();
#endif // !SETUP
}


#ifndef PROGRESS

#endif//��PROGRESS
	//sendStatusUpdate(QStringLiteral("����λ�������"), 0);
	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	cout << "�˳��������ʱ��Ϊ" << totaltime << "�룡" << endl;

	//�����߳�
	threadStatus = false;

}

void ProcessThread::LeftScannerRun(stepName stepNum, pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult)
{
	cout << "stepNum = " << stepNum << endl;
	//ͳ�Ƴ�������ʱ��
	clock_t start, finish;
	double totaltime;
	double startPosition;
	double endPosition;
	double position;
	//***��������**************************************************************//
	//������գ�Ϊ�´δ�����׼��
	leftRunDis.clear();
	//��ȡ������ز���
	emit sendStepNum(stepNum);
	//��ȡ���쵱ǰλ��
#ifndef SETUP
	startPosition = motion->GetCurrentPositon();
#endif //SETUP

	//�������ͼ
	//HObject inputImageFirst;
	//��������
	double speed;
	Vector<Point3f> points;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
	start = clock();

#ifndef SETUP
	//��ɾ�߼�������
	//serial->clearGratingData();
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
	cout << "1111111111" << endl;
	//�������
	cap->sendPLC();
	cout << "1111111111" << endl;
	//�������쿪ʼ�˶�, �ƶ�����Ϊ50mm
	motion->setLeadRailMotionDistance(CameraScanDistance);
	cout << "1111111111" << endl;
	GrabImageAsync(&inputImageFirst, cap->hv_AcqHandle, -1);
	cout << "1111111111" << endl;
#endif //SETUP

#ifndef READIMAGE
	QString ImageFileName;
	switch (stepNum)
	{
		//case left_hOneFP:
		//	ImageFileName = "edge/LeftFirst" + QString::number(imageNum) + ".tif";
		//	break;
		//case left_hTwoFP:
		//	ImageFileName = "edge/LeftFirst" + QString::number(imageNum) + ".tif";
		//	break;
		//case left_hTwoSP:
		//	ImageFileName = "edge/hole1.tif";
		//	break;
		//case left_hThreeFP:
		//	ImageFileName = "edge/LeftFirst" + QString::number(imageNum) + ".tif";
		//	break;
		//case left_hThreeSP:
		//	ImageFileName = "edge/hole1.tif";
		//	break;
		//case left_hFourFP:
		//	ImageFileName = "gear/LeftFirst" + QString::number(imageNum) + ".tif";
		//	break;
		//case left_hFourSP:
		//	ImageFileName = "gear/LeftFirst" + QString::number(imageNum) + ".tif";
		//	break;
		//case left_hFiveFP:
		//	ImageFileName = "gear/LeftFirst" + QString::number(imageNum) + ".tif";
		//	break;
		//case left_hFiveSP:
		//	ImageFileName = "gear/LeftFirst" + QString::number(imageNum) + ".tif";
		//	break;
		//case left_vOneFP:
		//	ImageFileName = "edge/LeftFirst" + QString::number(imageNum) + ".tif";
		//	break;
		//case left_vTwoFP:
		//	ImageFileName = "edge/LeftFirst" + QString::number(imageNum) + ".tif";
		//	break;
		//case left_vTwoSP:
		//	ImageFileName = "edge/hole1.tif";
		//	break;
		//case left_vThreeFP:
		//	ImageFileName = "gear/LeftFirst" + QString::number(imageNum) + ".tif";
		//	break;
		//case left_vThreeSP:
		//	ImageFileName = "gear/LeftFirst" + QString::number(imageNum) + ".tif";
		//	break;
		//default:
		//	break;
	}

	/*cout << ImageFileName.toStdString() << endl << endl;
	QByteArray ba = ImageFileName.toLatin1();
	const char *str = ba.data();
	cout << ImageFileName.toStdString() << endl << endl;
	HTuple  ImageName(str);
	*/
	//Mat img = imread("edge/LeftSecond1.tif");
#endif // !READIMAGE
	//��ʾ���ұ���ͼƬ
	emit sendImageAndSave(inputImageFirst, 101, "LeftFirst");
	////��ʾͼƬ
	//displayImage(inputImageFirst, 101);
	////����ͼƬ
	//SaveImage(inputImageFirst, "LeftFirst");
	//cout << "success" << endl;
	//ba = ImageFileName.toLatin1();
	//const char *str1 = ba.data();
	//HTuple  ImageName1(str1);

#ifndef SETUP
	if (scNum == 203 || scNum == 303 || scNum == 2030){
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
	//inputImageSecond = inputImageFirst;
	//}
	//else{
	//	HObject  ho_EmptyRegion;
	//	GenEmptyRegion(&ho_EmptyRegion);
	//	RegionToBin(ho_EmptyRegion, &inputImageSecond, 255, 0, 4096, 2000);
	//}
	//HalconCpp::WriteImage(inputImageFirst, "tiff", 0, "1.tif");
	//HalconCpp::WriteImage(inputImageSecond, "tiff", 0, "2.tif");
	emit sendImageAndSave(inputImageSecond, 102, "LeftSecond");
	////��ʾͼƬ
	//displayImage(inputImageSecond, 102);
	////����ͼƬ
	//SaveImage(inputImageSecond, "LeftSecond");
	cout << "success" << endl;
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
	cout << "����9" << endl;
#endif //SETUP

#ifndef PROGRESS

	////emit sendProcessingInformation(num, inputImageFirst, inputImageSecond);
	//switch (scNum)
	//{
	//case left_hOneFP:
	//	break;
	//case right_hOneFP:
	//	break;
	//case left_hOneSP:
	//	break;
	//case right_hOneSP:
	//	break;
	//case left_hTwoFP:
	//	break;
	//case right_hTwoFP:
	//	break;
	//case left_hTwoSP:
	//	break;
	//case right_hTwoSP:
	//	break;
	//case left_hThreeFP:
	//	break;
	//case right_hThreeFP:
	//	break;
	//case left_hThreeSP:
	//	break;
	//case right_hThreeSP:
	//	break;
	//case left_hFourFP:
	//	break;
	//case right_hFourFP:
	//	break;
	//case left_hFourSP:
	//	break;
	//case right_hFourSP:
	//	break;
	//case left_hFiveFP:
	//	break;
	//case right_hFiveFP:
	//	break;
	//case left_hFiveSP:
	//	break;
	//case right_hFiveSP:
	//	break;
	//case left_vOneFP:
	//	break;
	//case right_vOneFP:
	//	break;
	//case left_vOneSP:
	//	break;
	//case right_vOneSP:
	//	break;
	//case left_vTwoFP:
	//	break;
	//case right_vTwoFP:
	//	break;
	//case left_vTwoSP:
	//	break;
	//case right_vTwoSP:
	//	break;
	//case left_vThreeFP:
	//	break;
	//case right_vThreeFP:
	//	break;
	//case left_vThreeSP:
	//	break;
	//case right_vThreeSP:
	//	break;
	//default:
	//	break;
	//}
	switch (stepNum)
	{
	case 101:
		stepOneFirstLeftProcessing(101, inputImageFirst, inputImageSecond);
		break;
	case 201:
		stepTwoLeftProcessing(201, inputImageFirst, inputImageSecond);
		break;
	case 203:
		stepTwoLeftFlangeHoleProcessing(203, inputImageFirst);
		break;
	case 301:
		stepThreeLeftProcessing(301, inputImageFirst, inputImageSecond);
		break;
	case 303:
		stepThreeLeftFlangeHoleProcessing(303, inputImageFirst);
		break;
	case 401:
		stepFourLeftProcessing(401, inputImageFirst, inputImageSecond);
		break;
	case 501:
		stepFiveLeftProcessing(501, inputImageFirst, inputImageSecond);
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

void ProcessThread::RightScannerRun(stepName stepNum, pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult)
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
	emit sendStepNum(stepNum);
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
	switch (stepNum)
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
	switch (stepNum)
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
	if (stepNum != 204 && stepNum != 304 && stepNum != 2040){
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
	//emit sendProcessingInformation(num, inputImageFirst, inputImageSecond);
	switch (stepNum)
	{
	case 102:
		stepOneSecondRightProcessing(102, inputImageFirst, inputImageSecond);
		break;
	case 103:
		stepOneSecondRightProcessing(103, inputImageFirst, inputImageSecond);
		break;
	case 202:
		stepTwoRightProcessing(202, inputImageFirst, inputImageSecond);
		break;
	case 204:
		stepTwoRightFlangeHoleProcessing(204, inputImageFirst);
		break;
	case 302:
		stepThreeRightProcessing(302, inputImageFirst, inputImageSecond);
		break;
	case 304:
		stepThreeRightFlangeHoleProcessing(304, inputImageFirst);
		break;
	case 402:
		stepFourRightProcessing(402, inputImageFirst, inputImageSecond);
		break;
	case 502:
		stepFiveRightProcessing(502, inputImageFirst, inputImageSecond);
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
//H101 ��ⷢ����ֹ�ڷ�ʸ
void ProcessThread::stepOneFirstLeftProcessing(int num, HObject &inputImageFirst, HObject &inputImageSecond)
{
	cout << "in" << endl;
	
	CoreAlgorithm *alg_t = new CoreAlgorithm(0);

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
	cout << LeftIncrementFirst << "," << LeftIncrementSecond << endl;
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
	planeFirstImage = alg_t->getPlaneImage(inputImageFirst, status_planeFirst);
	planeSecondImage = alg_t->getPlaneImage(inputImageSecond, status_planeSecond);
	//��ȡ��Ե����ͼƬ
	//circleFirstImage = alg->getFlangeEdgeImage(inputImageFirst, status_circleFirst);
	//circleSecondImage = alg->getFlangeEdgeImage(inputImageSecond, status_circleSecond);
	//��ȡƽ��㼯
	planePointsFirst = alg_t->depthMapCalibration(planeFirstImage, status_planeFirst);
	planePointsSecond = alg_t->depthMapCalibration(planeSecondImage, status_planeSecond);
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
		planeparam = alg_t->getPlaneNormal(plane_cloudPoints);
		

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
//H102 ���ǰ������ֹ����Բ��1
void ProcessThread::stepOneSecondRightProcessing(int num, HObject &inputImageFirst, HObject &inputImageSecond)
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
//H201 ��ⷢ����ֹ��ƽ�淨ʸ��Բ������
void ProcessThread::stepTwoLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond)
{
	//�����ͷ����ϵ��Y������
	double LeftIncrementFirst;
	double LeftIncrementSecond;
	/*LeftIncrementFirst = ui.LeftRunFirst_StepTwo->text().toDouble();
	LeftIncrementSecond = ui.LeftRunFirst_StepTwo->text().toDouble() + ui.LeftCameraScan_StepTwo->text().toDouble()
		+ ui.LeftRunSecond_StepTwo->text().toDouble();*/
	vp->calculateMoveDistance(num,LeftIncrementFirst,LeftIncrementSecond);

#ifndef SETUP
	LeftIncrementFirst = leftRunDis[0];
	LeftIncrementSecond = leftRunDis[1];
#endif//SETUP
	QString runtime = vp->ReturnTime();
	QString runfileName = vp->engineNumFileName + "/outputFile/stepTwo/GratingData.csv";//д���ļ���Ŀ¼
	QFile runfile(runfileName);
	runfile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
	QTextStream runout(&runfile);
	runout << runtime << "," << "left" << "," << LeftIncrementFirst << "," << LeftIncrementSecond << "\n";
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
	if (num == 201){
		circleFirstImage = alg->getFlangeEdgeImage(inputImageFirst, status_circleFirst);
		circleSecondImage = alg->getFlangeEdgeImage(inputImageSecond, status_circleSecond);
	}
	if (num == 2010){
		circleFirstImage = alg->getFlangeEdgeImage(inputImageFirst, status_circleFirst);
		circleSecondImage = alg->getFlangeEdgeImage(inputImageSecond, status_circleSecond);
	}
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
		unifyPlanePoints.push_back(Point3f(planePointsFirst[i].x, planePointsFirst[i].y + LeftIncrementFirst, planePointsFirst[i].z));
	}

	for (int i = 0; i < planePointsSecond.size(); i++){
		unifyPlanePoints.push_back(Point3f(planePointsSecond[i].x, planePointsSecond[i].y + LeftIncrementSecond, planePointsSecond[i].z));
	}

	for (int i = 0; i < circlePointsFirst.size(); i++){
		unifyCirclePoints.push_back(Point3f(circlePointsFirst[i].x, circlePointsFirst[i].y + LeftIncrementFirst, circlePointsFirst[i].z));
	}

	for (int i = 0; i < circlePointsSecond.size(); i++){
		unifyCirclePoints.push_back(Point3f(circlePointsSecond[i].x, circlePointsSecond[i].y + LeftIncrementSecond, circlePointsSecond[i].z));
	}
	cout << "ͳһ�����ƽ��㼯������" << unifyPlanePoints.size() << endl;
	cout << "ͳһ������Ե�㼯������" << unifyCirclePoints.size() << endl;
	QString time = vp->ReturnTime();
	QString fileName;
	fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/engineUnifyCirclePoints/" + time + " engineUnifyCirclePoints.txt";
	QFile file(fileName);
	file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
	QTextStream out(&file);
	for (int i = 0; i < unifyCirclePoints.size(); i++){
		out << num << unifyCirclePoints[i].x << " " << unifyCirclePoints[i].y << " " << unifyCirclePoints[i].z << endl;
	}
	file.close();
	if (unifyPlanePoints.size() != 0)
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
		if (unifyCirclePoints.size() != 0)
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
			fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/engineCirclePointsThroughDistanceThreshold/" + time
				+ " engineCirclePointsThroughDistanceThreshold.txt";
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
			fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/engineFilterBorderPoints/" + time
				+ " engineFilterBorderPoints.txt";
			outfile.open(fileName.toStdString());
			for (int i = 0; i < filterBorderPoints.size(); i++){
				outfile << filterBorderPoints[i].x << " " << filterBorderPoints[i].y << " " << filterBorderPoints[i].z << endl;
			}
			outfile.close();
			//����任
			filterBorderPoints = vp->MeasureToRobotTool(filterBorderPoints);
			centerResult = vp->MeasureToRobotTool(centerResult);
			planeparam = vp->MeasureToRobotTool(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//����任
			filterBorderPoints = vp->RobotToolToBase(filterBorderPoints);
			centerResult = vp->RobotToolToBase(centerResult);
			planeparam = vp->RobotToolToBase(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//���洦����Ϣ
			QString cirlceStatusStr;
			if (abs(averageDistance - R) < 0.1){
				centerResult.status = true;
				cirlceStatusStr = "true";
			}
			else{
				centerResult.status = false;
				cirlceStatusStr = "false";
			}
			time = vp->ReturnTime();
			fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/engineParam.csv";//д���ļ���Ŀ¼
			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time << "," << num << "," << centerResult.center.x << "," << centerResult.center.y << "," << centerResult.center.z << "," << cirlceStatusStr << ","
				<< planeparam.A << "," << planeparam.B << "," << planeparam.C << ","
				<< averageDistance << "\n";
			file.close();
			//����������
			fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/engineFinalBorderPoints/" + time
				+ " engineFinalBorderPoints.txt";
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
			time = vp->ReturnTime();
			fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/engineParam.csv";//д���ļ���Ŀ¼
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
		fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/engineParam.csv";//д���ļ���Ŀ¼
		QFile file(fileName);
		file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
		QTextStream out(&file);
		out << time << "," << num << "," << "null" /*Բ��X*/ << "," << "null" /*Բ��Y*/ << "," << "null"/*Բ��Z*/ << "," << "false"/*Բ�����״̬*/ << ","
			<< "null"/*��ʸA*/ << "," << "null"/*��ʸB*/ << "," << "null"/*��ʸC*/ << "," << "null" /*��Ե�㵽Բ�ĵ�ƽ������*/ << "\n";
		file.close();
		emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
	}
}
//H202 ���ǰ������ֹ��ƽ�淨ʸ��Բ������
void ProcessThread::stepTwoRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond)
{
	//�����ͷ����ϵ��Y������
	double RightIncrementFirst;
	double RightIncrementSecond;
	/*RightIncrementFirst = ui.RightRunFirst_StepTwo->text().toDouble();
	RightIncrementSecond = ui.RightRunFirst_StepTwo->text().toDouble() + ui.RightCameraScan_StepTwo->text().toDouble()
	+ ui.RightRunSecond_StepTwo->text().toDouble();*/
	vp->calculateMoveDistance(num, RightIncrementFirst, RightIncrementSecond);
#ifndef SETUP
	RightIncrementFirst = rightRunDis[0];
	RightIncrementSecond = rightRunDis[1];
#endif//SETUP
	QString runtime = vp->ReturnTime();
	QString runfileName = vp->engineNumFileName + "/outputFile/stepTwo/GratingData.csv";//д���ļ���Ŀ¼
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
	if (num == 202){
		circleFirstImage = alg->getFlangeEdgeImage(inputImageFirst, status_circleFirst);
		circleSecondImage = alg->getFlangeEdgeImage(inputImageSecond, status_circleSecond);
	}
	if (num == 2020){
		circleFirstImage = alg->getFlangeEdgeImage(inputImageFirst, status_circleFirst);
		circleSecondImage = alg->getFlangeEdgeImage(inputImageSecond, status_circleSecond);
	}
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
	if (num == 202){
		fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/frontDriveBoxUnifyCirclePoints/" + time + " frontDriveBoxUnifyCirclePoints.txt";
	}
	if (num == 2020){
		fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/syntheticalDriveBoxUnifyCirclePoints/" + time + " syntheticalDriveBoxUnifyCirclePoints.txt";
	}
	QFile file(fileName);
	file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
	QTextStream out(&file);
	for (int i = 0; i < unifyCirclePoints.size(); i++){
		out << unifyCirclePoints[i].x << " " << unifyCirclePoints[i].y << " " << unifyCirclePoints[i].z << endl;
	}
	file.close();

	if (unifyPlanePoints.size() != 0)
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
		if (unifyCirclePoints.size() != 0)
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
			if (num == 202){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/frontDriveBoxCirclePointsThroughDistanceThreshold/" + time
					+ " frontDriveBoxCirclePointsThroughDistanceThreshold.txt";
			}
			if (num == 2020){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/syntheticalDriveBoxCirclePointsThroughDistanceThreshold/" + time
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
			if (num == 202){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/frontDriveBoxFilterBorderPoints/" + time
					+ " frontDriveBoxFilterBorderPoints.txt";
			}
			if (num == 2020){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/syntheticalDriveBoxFilterBorderPoints/" + time
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
			//���洦����Ϣ
			QString cirlceStatusStr;
			if (abs(averageDistance - R) < 0.1){
				centerResult.status = true;
				cirlceStatusStr = "true";
			}
			else{
				centerResult.status = false;
				cirlceStatusStr = "false";
			}
			time = vp->ReturnTime();
			if (num == 202){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/frontDriveBoxParam.csv";//д���ļ���Ŀ¼
			}
			if (num == 2020){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/syntheticalDriveBoxParam.csv";//д���ļ���Ŀ¼
			}
			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time << "," << num << "," << centerResult.center.x << "," << centerResult.center.y << "," << centerResult.center.z << "," << cirlceStatusStr << ","
				<< planeparam.A << "," << planeparam.B << "," << planeparam.C << ","
				<< averageDistance << "\n";
			file.close();
			//����������
			if (num == 202){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/frontDriveBoxFinalBorderPoints/" + time
					+ " frontDriveBoxFinalBorderPoints.txt";
			}
			if (num == 2020){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/syntheticalDriveBoxFinalBorderPoints/" + time
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
		else
		{
			//�����ռ�����
			CoreAlgorithm::StereoCircle centerResult;
			time = vp->ReturnTime();
			if (num == 202){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/frontDriveBoxParam.csv";//д���ļ���Ŀ¼
			}
			if (num == 2020){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/syntheticalDriveBoxParam.csv";//д���ļ���Ŀ¼
			}
			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time << "," << num << "," << "null" << "," << "null" << "," << "null" << "," << "false" << ","
				<< planeparam.A << "," << planeparam.B << "," << planeparam.C << ","
				<< "null" << "\n";
			file.close();
			bool planeparamStatus = true;
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
		if (num == 202){
			fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/frontDriveBoxParam.csv";//д���ļ���Ŀ¼
		}
		if (num == 2020){
			fileName = vp->engineNumFileName + "/outputFile/stepTwo/firstPart/syntheticalDriveBoxParam.csv";//д���ļ���Ŀ¼
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
//H203 ��ⷢ����ֹ��һ��������Բ������
void ProcessThread::stepTwoLeftFlangeHoleProcessing(int num, HObject inputImage)
{
	//�����ͷ����ϵ��Y������
	double LeftIncrementFirst, LeftIncrementSecond;
	/*LeftIncrementFirst = ui.LeftFangleHoleRun_StepTwo->text().toDouble();*/
	vp->calculateMoveDistance(num, LeftIncrementFirst, LeftIncrementSecond);
#ifndef SETUP
	LeftIncrementFirst = leftRunDis[0];
#endif//SETUP
	QString runtime = vp->ReturnTime();
	QString runfileName = vp->engineNumFileName + "/outputFile/stepTwo/GratingData.csv";//д���ļ���Ŀ¼
	QFile runfile(runfileName);
	runfile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
	QTextStream runout(&runfile);
	runout << runtime << "," << "left" << "," << LeftIncrementFirst << "\n";
	runfile.close();

	//����ƽ�洦��ͼƬ
	HObject planeFirstImage;
	//������Ե����ͼƬ
	HObject circleFirstImage;
	//����ƽ��㼯
	Vector<Point3f> planePointsFirst;
	//������Ե�㼯
	Vector<Point3f> circlePointsFirst;
	//����ͳһ����ϵ��ƽ��㼯
	Vector<Point3f> unifyPlanePoints;
	//����ͳһ����ϵ�±�Ե�㼯
	Vector<Point3f> unifyCirclePoints;
	//����ͼ������ true or false
	bool status_planeFirst;
	bool status_circleFirst;
	//��ȡƽ�洦��ͼƬ
	planeFirstImage = alg->getPlaneImage(inputImage, status_planeFirst);
	//��ȡ��Ե����ͼƬ
	if (num == 203){
		circleFirstImage = alg->getFlangeHoleEdgeImage(inputImage, status_circleFirst);
	}
	if (num == 2030){
		circleFirstImage = alg->getFlangeHoleEdgeImage(inputImage, status_circleFirst);
	}

	//��ȡƽ��㼯
	planePointsFirst = alg->depthMapCalibration(planeFirstImage, status_planeFirst);
	cout << "ƽ��㼯������" << planePointsFirst.size() << endl;
	//��ȡ��Ե�㼯
	circlePointsFirst = alg->depthMapCalibration(circleFirstImage, status_circleFirst);
	cout << "��Ե�㼯������" << circlePointsFirst.size() << endl;

	//�ɼ��㣬ͳһ����ϵ
	for (int i = 0; i < planePointsFirst.size(); i++){
		unifyPlanePoints.push_back(Point3f(planePointsFirst[i].x, planePointsFirst[i].y + LeftIncrementFirst, planePointsFirst[i].z));
	}

	for (int i = 0; i < circlePointsFirst.size(); i++){
		unifyCirclePoints.push_back(Point3f(circlePointsFirst[i].x, circlePointsFirst[i].y + LeftIncrementFirst, circlePointsFirst[i].z));
	}

	QString time = vp->ReturnTime();
	QString fileName;
	fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleUnifyCirclePoints/" + time + " engineFlangeHoleUnifyCirclePoints.txt";
	QFile file(fileName);
	file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
	QTextStream out(&file);
	for (int i = 0; i < unifyCirclePoints.size(); i++){
		out << unifyCirclePoints[i].x << " " << unifyCirclePoints[i].y << " " << unifyCirclePoints[i].z << endl;
	}
	file.close();
	if (unifyPlanePoints.size() != 0)
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
		if (unifyCirclePoints.size() != 0)
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
			fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleCirclePointsThroughDistanceThreshold/" + time
				+ " engineFlangeHoleCirclePointsThroughDistanceThreshold.txt";
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
			filterBorderPoints = alg->filterBorderPointsOnDistanceThrehold(subpoints, distances, R, 0.2);
			cout << "����ǰ���� = " << subpoints.size() << endl;
			cout << "���˺���� = " << filterBorderPoints.size() << endl;
			centerResult = alg->fitSteroCircle(filterBorderPoints, planeparam, R);
			cout << "�ռ�Բ�����꣺" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			averageDistance = alg->CalculateBorderPointsToCenterAverageDistance(filterBorderPoints, centerResult);
			cout << endl << "��Ե�㵽�ռ�Բ�ĵ�ƽ������Ϊ�� " << averageDistance << endl << endl;

			fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleFilterBorderPoints/" + time
				+ " engineFlangeHoleFilterBorderPoints.txt";
			outfile.open(fileName.toStdString());
			for (int i = 0; i < filterBorderPoints.size(); i++){
				outfile << filterBorderPoints[i].x << " " << filterBorderPoints[i].y << " " << filterBorderPoints[i].z << endl;
			}
			outfile.close();

			//����任
			filterBorderPoints = vp->MeasureToRobotTool(filterBorderPoints);
			centerResult = vp->MeasureToRobotTool(centerResult);
			planeparam = vp->MeasureToRobotTool(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//����任
			filterBorderPoints = vp->RobotToolToBase(filterBorderPoints);
			centerResult = vp->RobotToolToBase(centerResult);
			planeparam = vp->RobotToolToBase(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//���洦����Ϣ
			QString cirlceStatusStr;
			if (abs(averageDistance - R) < 0.1){
				centerResult.status = true;
				cirlceStatusStr = "true";
			}
			else{
				centerResult.status = false;
				cirlceStatusStr = "false";
			}
			time = vp->ReturnTime();
			fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleParam.csv";//д���ļ���Ŀ¼
			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time << "," << num << "," << centerResult.center.x << "," << centerResult.center.y << "," << centerResult.center.z << "," << cirlceStatusStr << ","
				<< planeparam.A << "," << planeparam.B << "," << planeparam.C << ","
				<< averageDistance << "\n";
			file.close();
			//����������
			fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleFinalBorderPoints/" + time
				+ " engineFlangeHoleFinalBorderPoints.txt";
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
		else
		{
			CoreAlgorithm::StereoCircle centerResult;
			centerResult.status = false;
			bool planeparamStatus = true;
			time = vp->ReturnTime();
			fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleParam.csv";//д���ļ���Ŀ¼
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
		fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleParam.csv";//д���ļ���Ŀ¼
		QFile file(fileName);
		file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
		QTextStream out(&file);
		out << time << "," << num << "," << "null" /*Բ��X*/ << "," << "null" /*Բ��Y*/ << "," << "null"/*Բ��Z*/ << "," << "false"/*Բ�����״̬*/ << ","
			<< "null"/*��ʸA*/ << "," << "null"/*��ʸB*/ << "," << "null"/*��ʸC*/ << "," << "null" /*��Ե�㵽Բ�ĵ�ƽ������*/ << "\n";
		file.close();
		emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
	}
}
//H204 ���ǰ������ֹ��һ��������Բ������
void ProcessThread::stepTwoRightFlangeHoleProcessing(int num, HObject inputImage)
{
	//�����ͷ����ϵ��Y������
	double RightIncrementFirst, RightIncrementSecond;
	/*RightIncrementFirst = ui.RightFangleHoleRun_StepTwo->text().toDouble();*/
	vp->calculateMoveDistance(num, RightIncrementFirst, RightIncrementSecond);
#ifndef SETUP
	RightIncrementFirst = rightRunDis[0];
#endif//SETUP
	QString runtime = vp->ReturnTime();
	QString runfileName = vp->engineNumFileName + "/outputFile/stepTwo/GratingData.csv";//д���ļ���Ŀ¼
	QFile runfile(runfileName);
	runfile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
	QTextStream runout(&runfile);
	runout << runtime << "," << "right" << "," << RightIncrementFirst << "\n";
	runfile.close();

	//����ƽ�洦��ͼƬ
	HObject planeFirstImage;
	//������Ե����ͼƬ
	HObject circleFirstImage;
	//����ƽ��㼯
	Vector<Point3f> planePointsFirst;
	//������Ե�㼯
	Vector<Point3f> circlePointsFirst;
	//����ͳһ����ϵ��ƽ��㼯
	Vector<Point3f> unifyPlanePoints;
	//����ͳһ����ϵ�±�Ե�㼯
	Vector<Point3f> unifyCirclePoints;
	//����ͼ������ true or false
	bool status_planeFirst;
	bool status_circleFirst;
	//��ȡƽ�洦��ͼƬ
	planeFirstImage = alg->getPlaneImage(inputImage, status_planeFirst);
	//��ȡ��Ե����ͼƬ
	if (num == 204){
		circleFirstImage = alg->getFlangeHoleEdgeImage(inputImage, status_circleFirst);
	}
	if (num == 2040){
		circleFirstImage = alg->getFlangeHoleEdgeImage(inputImage, status_circleFirst);
	}
	//��ȡƽ��㼯
	planePointsFirst = alg->depthMapCalibration(planeFirstImage, status_planeFirst);
	cout << "ƽ��㼯������" << planePointsFirst.size() << endl;
	//��ȡ��Ե�㼯
	circlePointsFirst = alg->depthMapCalibration(circleFirstImage, status_circleFirst);
	cout << "��Ե�㼯������" << circlePointsFirst.size() << endl;

	//�ɼ��㣬ͳһ����ϵ
	for (int i = 0; i < planePointsFirst.size(); i++){
		unifyPlanePoints.push_back(Point3f(planePointsFirst[i].x, planePointsFirst[i].y + RightIncrementFirst, planePointsFirst[i].z));
	}

	for (int i = 0; i < circlePointsFirst.size(); i++){
		unifyCirclePoints.push_back(Point3f(circlePointsFirst[i].x, circlePointsFirst[i].y + RightIncrementFirst, circlePointsFirst[i].z));
	}


	QString time = vp->ReturnTime();
	QString fileName;
	if (num == 204){
		fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/frontDriveBoxFlangeHoleUnifyCirclePoints/" + time
			+ " frontDriveBoxFlangeHoleUnifyCirclePoints.txt";
	}
	if (num == 2040){
		fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/syntheticalDriveBoxFlangeHoleUnifyCirclePoints/" + time
			+ " syntheticalDriveBoxFlangeHoleUnifyCirclePoints.txt";
	}
	QFile file(fileName);
	file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
	QTextStream out(&file);
	for (int i = 0; i < unifyCirclePoints.size(); i++){
		out << unifyCirclePoints[i].x << " " << unifyCirclePoints[i].y << " " << unifyCirclePoints[i].z << endl;
	}
	file.close();
	if (unifyPlanePoints.size() != 0)
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
		if (unifyPlanePoints.size() != 0)
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
			if (num == 204){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/frontDriveBoxFlangeHoleCirclePointsThroughDistanceThreshold/" + time
					+ " frontDriveBoxFlangeHoleCirclePointsThroughDistanceThreshold.txt";
			}
			if (num == 2040){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/syntheticalDriveBoxFlangeHoleCirclePointsThroughDistanceThreshold/" + time
					+ " syntheticalDriveBoxFlangeHoleCirclePointsThroughDistanceThreshold.txt";
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
			filterBorderPoints = alg->filterBorderPointsOnDistanceThrehold(subpoints, distances, R, 0.2);
			cout << "����ǰ���� = " << subpoints.size() << endl;
			cout << "���˺���� = " << filterBorderPoints.size() << endl;
			centerResult = alg->fitSteroCircle(filterBorderPoints, planeparam, R);
			cout << "�ռ�Բ�����꣺" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			averageDistance = alg->CalculateBorderPointsToCenterAverageDistance(filterBorderPoints, centerResult);
			cout << endl << "��Ե�㵽�ռ�Բ�ĵ�ƽ������Ϊ�� " << averageDistance << endl << endl;
			if (num == 204){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/frontDriveBoxFlangeHoleFilterBorderPoints/" + time
					+ " frontDriveBoxFlangeHoleFilterBorderPoints.txt";
			}
			if (num == 2040){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/syntheticalDriveBoxFlangeHoleFilterBorderPoints/" + time
					+ " syntheticalDriveBoxFlangeHoleFilterBorderPoints.txt";
			}
			outfile.open(fileName.toStdString());
			for (int i = 0; i < filterBorderPoints.size(); i++){
				outfile << filterBorderPoints[i].x << " " << filterBorderPoints[i].y << " " << filterBorderPoints[i].z << endl;
			}
			outfile.close();

			//����任
			//����任
			//�Ҳ�ͷ����ϵ�����ͷ����ϵ
			filterBorderPoints = vp->RightToLeft(filterBorderPoints);
			centerResult = vp->RightToLeft(centerResult);
			planeparam = vp->RightToLeft(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			filterBorderPoints = vp->MeasureToRobotTool(filterBorderPoints);
			centerResult = vp->MeasureToRobotTool(centerResult);
			planeparam = vp->MeasureToRobotTool(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			filterBorderPoints = vp->RobotToolToBase(filterBorderPoints);
			centerResult = vp->RobotToolToBase(centerResult);
			planeparam = vp->RobotToolToBase(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//���洦����Ϣ
			QString cirlceStatusStr;
			if (abs(averageDistance - R) < 0.1){
				centerResult.status = true;
				cirlceStatusStr = "true";
			}
			else{
				centerResult.status = false;
				cirlceStatusStr = "false";
			}
			time = vp->ReturnTime();
			if (num == 204){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/frontDriveBoxFlangeHoleParam.csv";//д���ļ���Ŀ¼
			}
			if (num == 2040){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/syntheticalDriveBoxFlangeHoleParam.csv";//д���ļ���Ŀ¼
			}
			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time << "," << num << "," << centerResult.center.x << "," << centerResult.center.y << "," << centerResult.center.z << "," << cirlceStatusStr << ","
				<< planeparam.A << "," << planeparam.B << "," << planeparam.C << ","
				<< averageDistance << "\n";
			file.close();
			//����������
			if (num == 204){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/frontDriveBoxFlangeHoleFinalBorderPoints/" + time
					+ " frontDriveBoxFlangeHoleFinalBorderPoints.txt";
			}
			if (num == 2040){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/syntheticalDriveBoxFlangeHoleFinalBorderPoints/" + time
					+ " syntheticalDriveBoxFlangeHoleFinalBorderPoints.txt";
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
		else
		{
			CoreAlgorithm::StereoCircle centerResult;
			centerResult.status = false;
			bool planeparamStatus = true;
			time = vp->ReturnTime();
			if (num == 204){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/frontDriveBoxFlangeHoleParam.csv";//д���ļ���Ŀ¼
			}
			if (num == 2040){
				fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/syntheticalDriveBoxFlangeHoleParam.csv";//д���ļ���Ŀ¼
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
		if (num == 204){
			fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/frontDriveBoxFlangeHoleParam.csv";//д���ļ���Ŀ¼
		}
		if (num == 2040){
			fileName = vp->engineNumFileName + "/outputFile/stepTwo/secondPart/syntheticalDriveBoxFlangeHoleParam.csv";//д���ļ���Ŀ¼
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
//H301 ����ۺϴ�����ֹ��ƽ�淨ʸ��Բ������
void ProcessThread::stepThreeLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond)
{
	//�����ͷ����ϵ��Y������
	double LeftIncrementFirst;
	double LeftIncrementSecond;
	//LeftIncrementFirst = ui.LeftRunFirst_StepThree->text().toDouble();
	//LeftIncrementSecond = ui.LeftRunFirst_StepThree->text().toDouble() + ui.LeftCameraScan_StepThree->text().toDouble()
	//	+ ui.LeftRunSecond_StepThree->text().toDouble();
	vp->calculateMoveDistance(num,LeftIncrementFirst,LeftIncrementSecond);
#ifndef SETUP
	LeftIncrementFirst = leftRunDis[0];
	LeftIncrementSecond = leftRunDis[1];
#endif//SETUP
	QString runtime = vp->ReturnTime();
	QString runfileName = vp->engineNumFileName + "/outputFile/stepThree/GratingData.csv";//д���ļ���Ŀ¼
	QFile runfile(runfileName);
	runfile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
	QTextStream runout(&runfile);
	runout << runtime << "," << "left" << "," << LeftIncrementFirst << "," << LeftIncrementSecond << "\n";
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
	circleFirstImage = alg->getFlangeEdgeImage(inputImageFirst, status_circleFirst);
	circleSecondImage = alg->getFlangeEdgeImage(inputImageSecond, status_circleSecond);
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
		unifyPlanePoints.push_back(Point3f(planePointsFirst[i].x, planePointsFirst[i].y + LeftIncrementFirst, planePointsFirst[i].z));
	}

	for (int i = 0; i < planePointsSecond.size(); i++){
		unifyPlanePoints.push_back(Point3f(planePointsSecond[i].x, planePointsSecond[i].y + LeftIncrementSecond, planePointsSecond[i].z));
	}

	for (int i = 0; i < circlePointsFirst.size(); i++){
		unifyCirclePoints.push_back(Point3f(circlePointsFirst[i].x, circlePointsFirst[i].y + LeftIncrementFirst, circlePointsFirst[i].z));
	}

	for (int i = 0; i < circlePointsSecond.size(); i++){
		unifyCirclePoints.push_back(Point3f(circlePointsSecond[i].x, circlePointsSecond[i].y + LeftIncrementSecond, circlePointsSecond[i].z));
	}
	cout << "ͳһ�����ƽ��㼯������" << unifyPlanePoints.size() << endl;
	cout << "ͳһ������Ե�㼯������" << unifyCirclePoints.size() << endl;
	QString time = vp->ReturnTime();
	QString fileName;
	fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxUnifyCirclePoints/" + time + " syntheticalDriveBoxUnifyCirclePoints.txt";
	QFile file(fileName);
	file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
	QTextStream out(&file);
	for (int i = 0; i < unifyCirclePoints.size(); i++){
		out << unifyCirclePoints[i].x << " " << unifyCirclePoints[i].y << " " << unifyCirclePoints[i].z << endl;
	}
	file.close();
	if (unifyPlanePoints.size() != 0)
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
		if (unifyCirclePoints.size() != 0)
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
			fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxCirclePointsThroughDistanceThreshold/" + time
				+ " syntheticalDriveBoxCirclePointsThroughDistanceThreshold.txt";
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

			fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxFilterBorderPoints/" + time
				+ " syntheticalDriveBoxFilterBorderPoints.txt";
			outfile.open(fileName.toStdString());
			for (int i = 0; i < filterBorderPoints.size(); i++){
				outfile << filterBorderPoints[i].x << " " << filterBorderPoints[i].y << " " << filterBorderPoints[i].z << endl;
			}
			outfile.close();

			//����任
			filterBorderPoints = vp->MeasureToRobotTool(filterBorderPoints);
			centerResult = vp->MeasureToRobotTool(centerResult);
			planeparam = vp->MeasureToRobotTool(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//����任
			filterBorderPoints = vp->RobotToolToBase(filterBorderPoints);
			centerResult = vp->RobotToolToBase(centerResult);
			planeparam = vp->RobotToolToBase(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//���洦����Ϣ
			QString cirlceStatusStr;
			if (abs(averageDistance - R) < 0.1){
				centerResult.status = true;
				cirlceStatusStr = "true";
			}
			else{
				centerResult.status = false;
				cirlceStatusStr = "false";
			}
			time = vp->ReturnTime();
			fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxParam.csv";//д���ļ���Ŀ¼
			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time << "," << num << "," << centerResult.center.x << "," << centerResult.center.y << "," << centerResult.center.z << "," << cirlceStatusStr << ","
				<< planeparam.A << "," << planeparam.B << "," << planeparam.C << ","
				<< averageDistance << "\n";
			file.close();
			//����������

			fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxFinalBorderPoints/" + time
				+ " syntheticalDriveBoxFinalBorderPoints.txt";
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
		else
		{
			CoreAlgorithm::StereoCircle centerResult;
			centerResult.status = false;
			bool planeparamStatus = true;
			time = vp->ReturnTime();
			fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxFinalBorderPoints/";//д���ļ���Ŀ¼
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
		fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxFinalBorderPoints/";//д���ļ���Ŀ¼
		QFile file(fileName);
		file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
		QTextStream out(&file);
		out << time << "," << num << "," << "null" /*Բ��X*/ << "," << "null" /*Բ��Y*/ << "," << "null"/*Բ��Z*/ << "," << "false"/*Բ�����״̬*/ << ","
			<< "null"/*��ʸA*/ << "," << "null"/*��ʸB*/ << "," << "null"/*��ʸC*/ << "," << "null" /*��Ե�㵽Բ�ĵ�ƽ������*/ << "\n";
		file.close();
		emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
	}
}
//H302 ���ǰ������ֹ��ƽ�淨ʸ��Բ������
void ProcessThread::stepThreeRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond)
{
	//�����ͷ����ϵ��Y������
	double RightIncrementFirst;
	double RightIncrementSecond;
	/*RightIncrementFirst = ui.RightRunFirst_StepThree->text().toDouble();
	RightIncrementSecond = ui.RightRunFirst_StepThree->text().toDouble() + ui.RightCameraScan_StepThree->text().toDouble()
	+ ui.RightRunSecond_StepThree->text().toDouble();*/
	vp->calculateMoveDistance(num,RightIncrementFirst,RightIncrementSecond);
#ifndef SETUP
	RightIncrementFirst = rightRunDis[0];
	RightIncrementSecond = rightRunDis[1];
#endif//SETUP
	QString runtime = vp->ReturnTime();
	QString runfileName = vp->engineNumFileName + "/outputFile/stepThree/GratingData.csv";//д���ļ���Ŀ¼
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
	circleFirstImage = alg->getFlangeEdgeImage(inputImageFirst, status_circleFirst);
	circleSecondImage = alg->getFlangeEdgeImage(inputImageSecond, status_circleSecond);
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
	fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/frontDriveBoxUnifyCirclePoints/" + time + " frontDriveBoxUnifyCirclePoints.txt";
	QFile file(fileName);
	file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
	QTextStream out(&file);
	for (int i = 0; i < unifyCirclePoints.size(); i++){
		out << unifyCirclePoints[i].x << " " << unifyCirclePoints[i].y << " " << unifyCirclePoints[i].z << endl;
	}
	file.close();

	if (unifyPlanePoints.size() != 0)
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
		if (unifyPlanePoints.size() != 0)
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
			fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/frontDriveBoxCirclePointsThroughDistanceThreshold/" + time
				+ " frontDriveBoxCirclePointsThroughDistanceThreshold.txt";
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

			fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/frontDriveBoxFilterBorderPoints/" + time
				+ " frontDriveBoxFilterBorderPoints.txt";
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
			//���洦����Ϣ
			QString cirlceStatusStr;
			if (abs(averageDistance - R) < 0.1){
				centerResult.status = true;
				cirlceStatusStr = "true";
			}
			else{
				centerResult.status = false;
				cirlceStatusStr = "false";
			}
			time = vp->ReturnTime();
			fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/frontDriveBoxParam.csv";//д���ļ���Ŀ¼
			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time << "," << num << "," << centerResult.center.x << "," << centerResult.center.y << "," << centerResult.center.z << "," << cirlceStatusStr << ","
				<< planeparam.A << "," << planeparam.B << "," << planeparam.C << ","
				<< averageDistance << "\n";
			file.close();
			//����������

			fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/frontDriveBoxFinalBorderPoints/" + time
				+ " frontDriveBoxFinalBorderPoints.txt";
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
		else
		{
			CoreAlgorithm::StereoCircle centerResult;
			centerResult.status = false;
			bool planeparamStatus = true;
			time = vp->ReturnTime();
			fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/frontDriveBoxParam.csv";//д���ļ���Ŀ¼
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
		fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/frontDriveBoxParam.csv";//д���ļ���Ŀ¼
		QFile file(fileName);
		file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
		QTextStream out(&file);
		out << time << "," << num << "," << "null" /*Բ��X*/ << "," << "null" /*Բ��Y*/ << "," << "null"/*Բ��Z*/ << "," << "false"/*Բ�����״̬*/ << ","
			<< "null"/*��ʸA*/ << "," << "null"/*��ʸB*/ << "," << "null"/*��ʸC*/ << "," << "null" /*��Ե�㵽Բ�ĵ�ƽ������*/ << "\n";
		file.close();
		emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
	}
}
//H303 ����ۺϴ�����ֹ��һ��������Բ������
void ProcessThread::stepThreeLeftFlangeHoleProcessing(int num, HObject inputImage)
{
	//�����ͷ����ϵ��Y������
	double LeftIncrementFirst, LeftIncrementSecond;
	/*LeftIncrementFirst = ui.LeftFangleHoleRun_StepThree->text().toDouble();*/
	vp->calculateMoveDistance(num, LeftIncrementFirst, LeftIncrementSecond);
#ifndef SETUP
	LeftIncrementFirst = leftRunDis[0];
#endif//SETUP
	QString runtime = vp->ReturnTime();
	QString runfileName = vp->engineNumFileName + "/outputFile/stepThree/GratingData.csv";//д���ļ���Ŀ¼
	QFile runfile(runfileName);
	runfile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
	QTextStream runout(&runfile);
	runout << runtime << "," << "left" << "," << LeftIncrementFirst << "\n";
	runfile.close();

	//����ƽ�洦��ͼƬ
	HObject planeFirstImage;
	//������Ե����ͼƬ
	HObject circleFirstImage;
	//����ƽ��㼯
	Vector<Point3f> planePointsFirst;
	//������Ե�㼯
	Vector<Point3f> circlePointsFirst;
	//����ͳһ����ϵ��ƽ��㼯
	Vector<Point3f> unifyPlanePoints;
	//����ͳһ����ϵ�±�Ե�㼯
	Vector<Point3f> unifyCirclePoints;
	//����ͼ������ true or false
	bool status_planeFirst;
	bool status_circleFirst;
	//��ȡƽ�洦��ͼƬ
	planeFirstImage = alg->getPlaneImage(inputImage, status_planeFirst);
	//��ȡ��Ե����ͼƬ
	circleFirstImage = alg->getFlangeHoleEdgeImage(inputImage, status_circleFirst);
	//��ȡƽ��㼯
	planePointsFirst = alg->depthMapCalibration(planeFirstImage, status_planeFirst);
	cout << "ƽ��㼯������" << planePointsFirst.size() << endl;
	//��ȡ��Ե�㼯
	circlePointsFirst = alg->depthMapCalibration(circleFirstImage, status_circleFirst);
	cout << "��Ե�㼯������" << circlePointsFirst.size() << endl;

	//�ɼ��㣬ͳһ����ϵ
	for (int i = 0; i < planePointsFirst.size(); i++){
		unifyPlanePoints.push_back(Point3f(planePointsFirst[i].x, planePointsFirst[i].y + LeftIncrementFirst, planePointsFirst[i].z));
	}

	for (int i = 0; i < circlePointsFirst.size(); i++){
		unifyCirclePoints.push_back(Point3f(circlePointsFirst[i].x, circlePointsFirst[i].y + LeftIncrementFirst, circlePointsFirst[i].z));
	}

	QString time = vp->ReturnTime();
	QString fileName;
	fileName = vp->engineNumFileName + "/outputFile/stepThree/secondPart/syntheticalDriveBoxFlangeHoleUnifyCirclePoints/" + time
		+ " syntheticalDriveBoxFlangeHoleUnifyCirclePoints.txt";
	QFile file(fileName);
	file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
	QTextStream out(&file);
	for (int i = 0; i < unifyCirclePoints.size(); i++){
		out << unifyCirclePoints[i].x << " " << unifyCirclePoints[i].y << " " << unifyCirclePoints[i].z << endl;
	}
	file.close();
	if (unifyPlanePoints.size() != 0)
	{
		//����ƽ����ƣ�PCL��
		pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
		//������Ե���ƣ�PCL��
		pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		//ƽ��㼯ת��ΪPCL����
		alg->threeDimensionToPCLPoints(unifyCirclePoints, plane_cloudPoints);
		//�����ռ�ƽ�淨ʸ���ݽṹ
		CoreAlgorithm::PlaneNormal planeparam;
		//��ȡƽ�淨ʸ
		planeparam = alg->getPlaneNormal(plane_cloudPoints);
		if (unifyCirclePoints.size() != 0)
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
			fileName = vp->engineNumFileName + "/outputFile/stepThree/secondPart/syntheticalDriveBoxFlangeHoleCirclePointsThroughDistanceThreshold/" + time
				+ " syntheticalDriveBoxFlangeHoleCirclePointsThroughDistanceThreshold.txt";
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
			filterBorderPoints = alg->filterBorderPointsOnDistanceThrehold(subpoints, distances, R, 0.2);
			cout << "����ǰ���� = " << subpoints.size() << endl;
			cout << "���˺���� = " << filterBorderPoints.size() << endl;
			centerResult = alg->fitSteroCircle(filterBorderPoints, planeparam, R);
			cout << "�ռ�Բ�����꣺" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			averageDistance = alg->CalculateBorderPointsToCenterAverageDistance(filterBorderPoints, centerResult);
			cout << endl << "��Ե�㵽�ռ�Բ�ĵ�ƽ������Ϊ�� " << averageDistance << endl << endl;

			fileName = vp->engineNumFileName + "/outputFile/stepThree/secondPart/syntheticalDriveBoxFlangeHoleFilterBorderPoints/" + time
				+ " syntheticalDriveBoxFlangeHoleFilterBorderPoints.txt";
			outfile.open(fileName.toStdString());
			for (int i = 0; i < filterBorderPoints.size(); i++){
				outfile << filterBorderPoints[i].x << " " << filterBorderPoints[i].y << " " << filterBorderPoints[i].z << endl;
			}
			outfile.close();

			//����任
			filterBorderPoints = vp->MeasureToRobotTool(filterBorderPoints);
			centerResult = vp->MeasureToRobotTool(centerResult);
			planeparam = vp->MeasureToRobotTool(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//����任
			filterBorderPoints = vp->RobotToolToBase(filterBorderPoints);
			centerResult = vp->RobotToolToBase(centerResult);
			planeparam = vp->RobotToolToBase(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//���洦����Ϣ
			QString cirlceStatusStr;
			if (abs(averageDistance - R) < 0.1){
				centerResult.status = true;
				cirlceStatusStr = "true";
			}
			else{
				centerResult.status = false;
				cirlceStatusStr = "false";
			}
			time = vp->ReturnTime();
			fileName = vp->engineNumFileName + "/outputFile/stepThree/secondPart/syntheticalDriveBoxFlangeHoleParam.csv";//д���ļ���Ŀ¼
			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time << "," << num << "," << centerResult.center.x << "," << centerResult.center.y << "," << centerResult.center.z << "," << cirlceStatusStr << ","
				<< planeparam.A << "," << planeparam.B << "," << planeparam.C << ","
				<< averageDistance << "\n";
			file.close();
			//����������

			fileName = vp->engineNumFileName + "/outputFile/stepThree/secondPart/syntheticalDriveBoxFlangeHoleFinalBorderPoints/" + time
				+ " syntheticalDriveBoxFlangeHoleFinalBorderPoints.txt";
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
		else
		{
			CoreAlgorithm::StereoCircle centerResult;
			centerResult.status = false;
			bool planeparamStatus = true;
			time = vp->ReturnTime();
			fileName = vp->engineNumFileName + "/outputFile/stepThree/secondPart/syntheticalDriveBoxFlangeHoleParam.csv";//д���ļ���Ŀ¼
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
		fileName = vp->engineNumFileName + "/outputFile/stepThree/secondPart/syntheticalDriveBoxFlangeHoleParam.csv";//д���ļ���Ŀ¼
		QFile file(fileName);
		file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
		QTextStream out(&file);
		out << time << "," << num << "," << "null" /*Բ��X*/ << "," << "null" /*Բ��Y*/ << "," << "null"/*Բ��Z*/ << "," << "false"/*Բ�����״̬*/ << ","
			<< "null"/*��ʸA*/ << "," << "null"/*��ʸB*/ << "," << "null"/*��ʸC*/ << "," << "null" /*��Ե�㵽Բ�ĵ�ƽ������*/ << "\n";
		file.close();
		emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
	}
}
//H304 ���ǰ������ֹ��һ��������Բ������
void ProcessThread::stepThreeRightFlangeHoleProcessing(int num, HObject inputImage)
{
	//�����ͷ����ϵ��Y������
	double RightIncrementFirst, RightIncrementSecond;
	/*RightIncrementFirst = ui.RightFangleHoleRun_StepThree->text().toDouble();;*/
	vp->calculateMoveDistance(num,RightIncrementFirst,RightIncrementSecond);
#ifndef SETUP
	RightIncrementFirst = rightRunDis[0];
#endif//SETUP
	QString runtime = vp->ReturnTime();
	QString runfileName = vp->engineNumFileName + "/outputFile/stepThree/GratingData.csv";//д���ļ���Ŀ¼
	QFile runfile(runfileName);
	runfile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
	QTextStream runout(&runfile);
	runout << runtime << "," << "right" << "," << RightIncrementFirst << "\n";
	runfile.close();
	//����ƽ�洦��ͼƬ
	HObject planeFirstImage;
	//������Ե����ͼƬ
	HObject circleFirstImage;
	//����ƽ��㼯
	Vector<Point3f> planePointsFirst;
	//������Ե�㼯
	Vector<Point3f> circlePointsFirst;
	//����ͳһ����ϵ��ƽ��㼯
	Vector<Point3f> unifyPlanePoints;
	//����ͳһ����ϵ�±�Ե�㼯
	Vector<Point3f> unifyCirclePoints;
	//����ͼ������ true or false
	bool status_planeFirst;
	bool status_circleFirst;
	//��ȡƽ�洦��ͼƬ
	planeFirstImage = alg->getPlaneImage(inputImage, status_planeFirst);
	//��ȡ��Ե����ͼƬ
	circleFirstImage = alg->getFlangeHoleEdgeImage(inputImage, status_circleFirst);
	//��ȡƽ��㼯
	planePointsFirst = alg->depthMapCalibration(planeFirstImage, status_planeFirst);
	cout << "ƽ��㼯������" << planePointsFirst.size() << endl;
	//��ȡ��Ե�㼯
	circlePointsFirst = alg->depthMapCalibration(circleFirstImage, status_circleFirst);
	cout << "��Ե�㼯������" << circlePointsFirst.size() << endl;

	//�ɼ��㣬ͳһ����ϵ
	for (int i = 0; i < planePointsFirst.size(); i++){
		unifyPlanePoints.push_back(Point3f(planePointsFirst[i].x, planePointsFirst[i].y + RightIncrementFirst, planePointsFirst[i].z));
	}

	for (int i = 0; i < circlePointsFirst.size(); i++){
		unifyCirclePoints.push_back(Point3f(circlePointsFirst[i].x, circlePointsFirst[i].y + RightIncrementFirst, circlePointsFirst[i].z));
	}


	QString time = vp->ReturnTime();
	QString fileName;
	fileName = vp->engineNumFileName + "/outputFile/stepThree/secondPart/frontDriveBoxFlangeHoleUnifyCirclePoints/" + time + " frontDriveBoxFlangeHoleUnifyCirclePoints.txt";
	QFile file(fileName);
	file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
	QTextStream out(&file);
	for (int i = 0; i < unifyCirclePoints.size(); i++){
		out << unifyCirclePoints[i].x << " " << unifyCirclePoints[i].y << " " << unifyCirclePoints[i].z << endl;
	}
	file.close();
	if (unifyPlanePoints.size() != 0)
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
		if (unifyCirclePoints.size() != 0)
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
			fileName = vp->engineNumFileName + "/outputFile/stepThree/secondPart/frontDriveBoxFlangeHoleCirclePointsThroughDistanceThreshold/" + time
				+ " frontDriveBoxFlangeHoleCirclePointsThroughDistanceThreshold.txt";
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
			filterBorderPoints = alg->filterBorderPointsOnDistanceThrehold(subpoints, distances, R, 0.2);
			cout << "����ǰ���� = " << subpoints.size() << endl;
			cout << "���˺���� = " << filterBorderPoints.size() << endl;
			centerResult = alg->fitSteroCircle(filterBorderPoints, planeparam, R);
			cout << "�ռ�Բ�����꣺" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			averageDistance = alg->CalculateBorderPointsToCenterAverageDistance(filterBorderPoints, centerResult);
			cout << endl << "��Ե�㵽�ռ�Բ�ĵ�ƽ������Ϊ�� " << averageDistance << endl << endl;

			fileName = vp->engineNumFileName + "/outputFile/stepThree/secondPart/frontDriveBoxFlangeHoleFilterBorderPoints/" + time
				+ " frontDriveBoxFlangeHoleFilterBorderPoints.txt";
			outfile.open(fileName.toStdString());
			for (int i = 0; i < filterBorderPoints.size(); i++){
				outfile << filterBorderPoints[i].x << " " << filterBorderPoints[i].y << " " << filterBorderPoints[i].z << endl;
			}
			outfile.close();

			//����任
			//����任
			//�Ҳ�ͷ����ϵ�����ͷ����ϵ
			filterBorderPoints = vp->RightToLeft(filterBorderPoints);
			centerResult = vp->RightToLeft(centerResult);
			planeparam = vp->RightToLeft(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			filterBorderPoints = vp->MeasureToRobotTool(filterBorderPoints);
			centerResult = vp->MeasureToRobotTool(centerResult);
			planeparam = vp->MeasureToRobotTool(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			filterBorderPoints = vp->RobotToolToBase(filterBorderPoints);
			centerResult = vp->RobotToolToBase(centerResult);
			planeparam = vp->RobotToolToBase(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//���洦����Ϣ
			QString cirlceStatusStr;
			if (abs(averageDistance - R) < 0.1){
				centerResult.status = true;
				cirlceStatusStr = "true";
			}
			else{
				centerResult.status = false;
				cirlceStatusStr = "false";
			}
			time = vp->ReturnTime();
			fileName = vp->engineNumFileName + "/outputFile/stepThree/secondPart/frontDriveBoxFlangeHoleParam.csv";//д���ļ���Ŀ¼
			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time << "," << num << "," << centerResult.center.x << "," << centerResult.center.y << "," << centerResult.center.z << "," << cirlceStatusStr << ","
				<< planeparam.A << "," << planeparam.B << "," << planeparam.C << ","
				<< averageDistance << "\n";
			file.close();
			//����������

			fileName = vp->engineNumFileName + "/outputFile/stepThree/secondPart/frontDriveBoxFlangeHoleFinalBorderPoints/" + time
				+ " frontDriveBoxFlangeHoleFinalBorderPoints.txt";
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
		else
		{
			CoreAlgorithm::StereoCircle centerResult;
			centerResult.status = false;
			bool planeparamStatus = true;
			time = vp->ReturnTime();
			fileName = vp->engineNumFileName + "/outputFile/stepThree/secondPart/frontDriveBoxFlangeHoleParam.csv";//д���ļ���Ŀ¼
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
		fileName = vp->engineNumFileName + "/outputFile/stepThree/secondPart/frontDriveBoxFlangeHoleParam.csv";//д���ļ���Ŀ¼
		QFile file(fileName);
		file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
		QTextStream out(&file);
		out << time << "," << num << "," << "null" /*Բ��X*/ << "," << "null" /*Բ��Y*/ << "," << "null"/*Բ��Z*/ << "," << "false"/*Բ�����״̬*/ << ","
			<< "null"/*��ʸA*/ << "," << "null"/*��ʸB*/ << "," << "null"/*��ʸC*/ << "," << "null" /*��Ե�㵽Բ�ĵ�ƽ������*/ << "\n";
		file.close();
		emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
	}
}
//H401 ����۴����仨���׵�ƽ�淨ʸ��Բ�� 
void ProcessThread::stepFourLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond)
{
	//�����ͷ����ϵ��Y������
	double LeftIncrementFirst;
	double LeftIncrementSecond;
	/*LeftIncrementFirst = ui.LeftRunFirst_StepFour->text().toDouble();
	LeftIncrementSecond = ui.LeftRunFirst_StepFour->text().toDouble() + ui.LeftCameraScan_StepFour->text().toDouble()
	+ ui.LeftRunSecond_StepFour->text().toDouble();*/
	vp->calculateMoveDistance(num, LeftIncrementFirst, LeftIncrementSecond);
#ifndef SETUP
	LeftIncrementFirst = leftRunDis[0];
	LeftIncrementSecond = leftRunDis[1];
#endif//SETUP
	QString runtime = vp->ReturnTime();
	QString runfileName;
	if (num == 401){
		runfileName = vp->engineNumFileName + "/outputFile/stepFour/GratingData.csv";//д���ļ���Ŀ¼
	}
	if (num == 3010){
		runfileName = vp->engineNumFileName + "/outputFile/stepThree/GratingData.csv";//д���ļ���Ŀ¼
	}
	QFile runfile(runfileName);
	runfile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
	QTextStream runout(&runfile);
	runout << runtime << "," << "left" << "," << LeftIncrementFirst << "," << LeftIncrementSecond << "\n";
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
	if (num == 401){
		circleFirstImage = alg->getEngineSplineEdgeImage(inputImageFirst, status_circleFirst);
		circleSecondImage = alg->getEngineSplineEdgeImage(inputImageSecond, status_circleSecond);
	}
	if (num == 3010){
		circleFirstImage = alg->getEngineSplineEdgeImage(inputImageFirst, status_circleFirst);
		circleSecondImage = alg->getEngineSplineEdgeImage(inputImageSecond, status_circleSecond);
	}

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
		unifyPlanePoints.push_back(Point3f(planePointsFirst[i].x, planePointsFirst[i].y + LeftIncrementFirst, planePointsFirst[i].z));
	}

	for (int i = 0; i < planePointsSecond.size(); i++){
		unifyPlanePoints.push_back(Point3f(planePointsSecond[i].x, planePointsSecond[i].y + LeftIncrementSecond, planePointsSecond[i].z));
	}

	for (int i = 0; i < circlePointsFirst.size(); i++){
		unifyCirclePoints.push_back(Point3f(circlePointsFirst[i].x, circlePointsFirst[i].y + LeftIncrementFirst, circlePointsFirst[i].z));
	}

	for (int i = 0; i < circlePointsSecond.size(); i++){
		unifyCirclePoints.push_back(Point3f(circlePointsSecond[i].x, circlePointsSecond[i].y + LeftIncrementSecond, circlePointsSecond[i].z));
	}
	cout << "ͳһ�����ƽ��㼯������" << unifyPlanePoints.size() << endl;
	cout << "ͳһ������Ե�㼯������" << unifyCirclePoints.size() << endl;
	QString time = vp->ReturnTime();
	QString fileName;
	if (num == 401){
		fileName = vp->engineNumFileName + "/outputFile/stepFour/firstPart/syntheticalDriveBoxUnifyCirclePoints/" + time + " syntheticalDriveBoxUnifyCirclePoints.txt";
	}
	if (num == 3010){
		fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/engineUnifyCirclePoints/" + time + " engineUnifyCirclePoints.txt";
	}
	QFile file(fileName);
	file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
	QTextStream out(&file);
	for (int i = 0; i < unifyCirclePoints.size(); i++){
		out << unifyCirclePoints[i].x << " " << unifyCirclePoints[i].y << " " << unifyCirclePoints[i].z << endl;
	}
	file.close();
	if (unifyPlanePoints.size() != 0)
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
		if (unifyPlanePoints.size() != 0)
		{
			//�����Ե�㵽��Ͽռ�ƽ��ľ��룬�����þ�����ֵ��ȡ������ֵ�ĵ�
			//�������������
			Vector<Point3f> out_circlePoints;
			//�����㵽ƽ����뼯��
			Vector<double> distance;
			//��������ֵΪmm����������ֵΪmm
			alg->setDistanceThresholdAndGetPoints(unifyCirclePoints, planeparam, distance, 0, 0.2, out_circlePoints);
			cout << "�������������" << out_circlePoints.size() << endl;
			ofstream outfile;
			if (num == 401){
				fileName = vp->engineNumFileName + "/outputFile/stepFour/firstPart/syntheticalDriveBoxCirclePointsThroughDistanceThreshold/" + time
					+ " syntheticalDriveBoxCirclePointsThroughDistanceThreshold.txt";
			}
			if (num == 3010){
				fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/engineCirclePointsThroughDistanceThreshold/" + time
					+ " engineCirclePointsThroughDistanceThreshold.txt";
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

			if (num == 401){
				fileName = vp->engineNumFileName + "/outputFile/stepFour/firstPart/syntheticalDriveBoxFilterBorderPoints/" + time
					+ " syntheticalDriveBoxFilterBorderPoints.txt";
			}
			if (num == 3010){
				fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/engineFilterBorderPoints/" + time
					+ " engineFilterBorderPoints.txt";
			}

			outfile.open(fileName.toStdString());
			for (int i = 0; i < filterBorderPoints.size(); i++){
				outfile << filterBorderPoints[i].x << " " << filterBorderPoints[i].y << " " << filterBorderPoints[i].z << endl;
			}
			outfile.close();

			//����任
			filterBorderPoints = vp->MeasureToRobotTool(filterBorderPoints);
			centerResult = vp->MeasureToRobotTool(centerResult);
			planeparam = vp->MeasureToRobotTool(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//����任
			filterBorderPoints = vp->RobotToolToBase(filterBorderPoints);
			centerResult = vp->RobotToolToBase(centerResult);
			planeparam = vp->RobotToolToBase(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//���洦����Ϣ
			QString cirlceStatusStr;
			if (abs(averageDistance - R) < 0.1){
				centerResult.status = true;
				cirlceStatusStr = "true";
			}
			else{
				centerResult.status = false;
				cirlceStatusStr = "false";
			}
			time = vp->ReturnTime();
			if (num == 401){
				fileName = vp->engineNumFileName + "/outputFile/stepFour/firstPart/syntheticalDriveBoxParam.csv";//д���ļ���Ŀ¼
			}
			if (num == 3010){
				fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/engineParam.csv";//д���ļ���Ŀ¼
			}

			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time << "," << num << "," << centerResult.center.x << "," << centerResult.center.y << "," << centerResult.center.z << "," << cirlceStatusStr << ","
				<< planeparam.A << "," << planeparam.B << "," << planeparam.C << ","
				<< averageDistance << "\n";
			file.close();
			//����������

			if (num == 401){
				fileName = vp->engineNumFileName + "/outputFile/stepFour/firstPart/syntheticalDriveBoxFinalBorderPoints/" + time
					+ " syntheticalDriveBoxFinalBorderPoints.txt";
			}
			if (num == 3010){
				fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/engineFinalBorderPoints/" + time
					+ " engineFinalBorderPoints.txt";
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
		else
		{
			CoreAlgorithm::StereoCircle centerResult;
			centerResult.status = false;
			bool planeparamStatus = true;
			time = vp->ReturnTime();
			if (num == 401){
				fileName = vp->engineNumFileName + "/outputFile/stepFour/firstPart/syntheticalDriveBoxParam.csv";//д���ļ���Ŀ¼
			}
			if (num == 3010){
				fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/engineParam.csv";//д���ļ���Ŀ¼
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
		if (num == 401){
			fileName = vp->engineNumFileName + "/outputFile/stepFour/firstPart/syntheticalDriveBoxParam.csv";//д���ļ���Ŀ¼
		}
		if (num == 3010){
			fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/engineParam.csv";//д���ļ���Ŀ¼
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
//H402 ���ǰ�����仨�����ƽ�淨ʸ��Բ�� 
void ProcessThread::stepFourRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond)
{

	//�����ͷ����ϵ��Y������
	double RightIncrementFirst;
	double RightIncrementSecond;
	/*RightIncrementFirst = ui.RightRunFirst_StepFour->text().toDouble();
	RightIncrementSecond = ui.RightRunFirst_StepFour->text().toDouble() + ui.RightCameraScan_StepFour->text().toDouble()
		+ ui.RightRunSecond_StepFour->text().toDouble();*/
	vp->calculateMoveDistance(num, RightIncrementFirst, RightIncrementSecond);
#ifndef SETUP
	RightIncrementFirst = rightRunDis[0];
	RightIncrementSecond = rightRunDis[1];
#endif//SETUP
	QString runtime = vp->ReturnTime();
	QString runfileName;
	if (num == 402){
		runfileName = vp->engineNumFileName + "/outputFile/stepFour/GratingData.csv";//д���ļ���Ŀ¼
	}
	if (num == 3020){
		runfileName = vp->engineNumFileName + "/outputFile/stepThree/GratingData.csv";//д���ļ���Ŀ¼
	}
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
	if (num == 402){
		circleFirstImage = alg->getfrontDriveBoxSplineEdgeImage(inputImageFirst, status_circleFirst);
		circleSecondImage = alg->getfrontDriveBoxSplineEdgeImage(inputImageSecond, status_circleSecond);
	}
	if (num == 3020){
		circleFirstImage = alg->getfrontDriveBoxSplineEdgeImage(inputImageFirst, status_circleFirst);
		circleSecondImage = alg->getfrontDriveBoxSplineEdgeImage(inputImageSecond, status_circleSecond);
	}

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
	if (num == 402){
		fileName = vp->engineNumFileName + "/outputFile/stepFour/firstPart/frontDriveBoxUnifyCirclePoints/" + time
			+ " frontDriveBoxUnifyCirclePoints.txt";
	}
	if (num == 3020){
		fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxUnifyCirclePoints/" + time
			+ " syntheticalDriveBoxUnifyCirclePoints.txt";
	}
	QFile file(fileName);
	file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
	QTextStream out(&file);
	for (int i = 0; i < unifyCirclePoints.size(); i++){
		out << unifyCirclePoints[i].x << " " << unifyCirclePoints[i].y << " " << unifyCirclePoints[i].z << endl;
	}
	file.close();
	if (unifyPlanePoints.size() != 0)
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
		if (unifyCirclePoints.size() != 0)
		{
			//�����Ե�㵽��Ͽռ�ƽ��ľ��룬�����þ�����ֵ��ȡ������ֵ�ĵ�
			//�������������
			Vector<Point3f> out_circlePoints;
			//�����㵽ƽ����뼯��
			Vector<double> distance;
			//��������ֵΪmm����������ֵΪmm
			alg->setDistanceThresholdAndGetPoints(unifyCirclePoints, planeparam, distance, 0, 0.2, out_circlePoints);
			cout << "�������������" << out_circlePoints.size() << endl;
			ofstream outfile;
			if (num == 402){
				fileName = vp->engineNumFileName + "/outputFile/stepFour/firstPart/frontDriveBoxCirclePointsThroughDistanceThreshold/" + time
					+ " frontDriveBoxCirclePointsThroughDistanceThreshold.txt";
			}
			if (num == 3020){
				fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxCirclePointsThroughDistanceThreshold/" + time
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
			filterBorderPoints = alg->filterBorderPointsOnDistanceThrehold(subpoints, distances, R, 0.2);
			cout << "����ǰ���� = " << subpoints.size() << endl;
			cout << "���˺���� = " << filterBorderPoints.size() << endl;
			centerResult = alg->fitSteroCircle(filterBorderPoints, planeparam, R);
			cout << "�ռ�Բ�����꣺" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			averageDistance = alg->CalculateBorderPointsToCenterAverageDistance(filterBorderPoints, centerResult);
			cout << endl << "��Ե�㵽�ռ�Բ�ĵ�ƽ������Ϊ�� " << averageDistance << endl << endl;

			if (num == 402){
				fileName = vp->engineNumFileName + "/outputFile/stepFour/firstPart/frontDriveBoxFilterBorderPoints/" + time
					+ " frontDriveBoxFilterBorderPoints.txt";
			}
			if (num == 3020){
				fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxFilterBorderPoints/" + time
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
			//���洦����Ϣ
			QString cirlceStatusStr;
			if (abs(averageDistance - R) < 0.1){
				centerResult.status = true;
				cirlceStatusStr = "true";
			}
			else{
				centerResult.status = false;
				cirlceStatusStr = "false";
			}
			time = vp->ReturnTime();
			if (num == 402){
				fileName = vp->engineNumFileName + "/outputFile/stepFour/firstPart/frontDriveBoxParam.csv";//д���ļ���Ŀ¼
			}
			if (num == 3020){
				fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxParam.csv";//д���ļ���Ŀ¼
			}
			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time << "," << num << "," << centerResult.center.x << "," << centerResult.center.y << "," << centerResult.center.z << "," << cirlceStatusStr << ","
				<< planeparam.A << "," << planeparam.B << "," << planeparam.C << ","
				<< averageDistance << "\n";
			file.close();
			//����������

			if (num == 402){
				fileName = vp->engineNumFileName + "/outputFile/stepFour/firstPart/frontDriveBoxFinalBorderPoints/" + time
					+ " frontDriveBoxFinalBorderPoints.txt";
			}
			if (num == 3020){
				fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxFinalBorderPoints/" + time
					+ " frontDriveBoxFinalBorderPoints.txt";
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
		else
		{
			CoreAlgorithm::StereoCircle centerResult;
			centerResult.status = false;
			bool planeparamStatus = true;
			time = vp->ReturnTime();
			if (num == 402){
				fileName = vp->engineNumFileName + "/outputFile/stepFour/firstPart/frontDriveBoxFinalBorderPoints/" + time
					+ " frontDriveBoxFinalBorderPoints.txt";
			}
			if (num == 3020){
				fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxFinalBorderPoints/" + time
					+ " frontDriveBoxFinalBorderPoints.txt";
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
		if (num == 402){
			fileName = vp->engineNumFileName + "/outputFile/stepFour/firstPart/frontDriveBoxFinalBorderPoints/" + time
				+ " frontDriveBoxFinalBorderPoints.txt";
		}
		if (num == 3020){
			fileName = vp->engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxFinalBorderPoints/" + time
				+ " frontDriveBoxFinalBorderPoints.txt";
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
//H501 ��ⷢ���������׵�ƽ�淨ʸ��Բ��
void ProcessThread::stepFiveLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond)
{
	//�����ͷ����ϵ��Y������
	double LeftIncrementFirst;
	double LeftIncrementSecond;
	/*LeftIncrementFirst = ui.LeftRunFirst_StepFive->text().toDouble();
	LeftIncrementSecond = ui.LeftRunFirst_StepFive->text().toDouble() + ui.LeftCameraScan_StepFive->text().toDouble()
	+ ui.LeftRunSecond_StepFive->text().toDouble();*/
	vp->calculateMoveDistance(num, LeftIncrementFirst, LeftIncrementSecond);
#ifndef SETUP
	LeftIncrementFirst = leftRunDis[0];
	LeftIncrementSecond = leftRunDis[1];
#endif//SETUP
	QString runtime = vp->ReturnTime();
	QString runfileName = vp->engineNumFileName + "/outputFile/stepFive/GratingData.csv";//д���ļ���Ŀ¼
	QFile runfile(runfileName);
	runfile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
	QTextStream runout(&runfile);
	runout << runtime << "," << "left" << "," << LeftIncrementFirst << "," << LeftIncrementSecond << "\n";
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
	circleFirstImage = alg->getEngineSplineEdgeImage(inputImageFirst, status_circleFirst);
	circleSecondImage = alg->getEngineSplineEdgeImage(inputImageSecond, status_circleSecond);
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
		unifyPlanePoints.push_back(Point3f(planePointsFirst[i].x, planePointsFirst[i].y + LeftIncrementFirst, planePointsFirst[i].z));
	}

	for (int i = 0; i < planePointsSecond.size(); i++){
		unifyPlanePoints.push_back(Point3f(planePointsSecond[i].x, planePointsSecond[i].y + LeftIncrementSecond, planePointsSecond[i].z));
	}

	for (int i = 0; i < circlePointsFirst.size(); i++){
		unifyCirclePoints.push_back(Point3f(circlePointsFirst[i].x, circlePointsFirst[i].y + LeftIncrementFirst, circlePointsFirst[i].z));
	}

	for (int i = 0; i < circlePointsSecond.size(); i++){
		unifyCirclePoints.push_back(Point3f(circlePointsSecond[i].x, circlePointsSecond[i].y + LeftIncrementSecond, circlePointsSecond[i].z));
	}
	cout << "ͳһ�����ƽ��㼯������" << unifyPlanePoints.size() << endl;
	cout << "ͳһ������Ե�㼯������" << unifyCirclePoints.size() << endl;
	QString time = vp->ReturnTime();
	QString fileName;
	fileName = vp->engineNumFileName + "/outputFile/stepFive/firstPart/engineUnifyCirclePoints/" + time + " engineUnifyCirclePoints.txt";
	QFile file(fileName);
	file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
	QTextStream out(&file);
	for (int i = 0; i < unifyCirclePoints.size(); i++){
		out << unifyCirclePoints[i].x << " " << unifyCirclePoints[i].y << " " << unifyCirclePoints[i].z << endl;
	}
	file.close();
	if (unifyPlanePoints.size() != 0)
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
		if (unifyCirclePoints.size() != 0)
		{
			//�����Ե�㵽��Ͽռ�ƽ��ľ��룬�����þ�����ֵ��ȡ������ֵ�ĵ�
			//�������������
			Vector<Point3f> out_circlePoints;
			//�����㵽ƽ����뼯��
			Vector<double> distance;
			//��������ֵΪmm����������ֵΪmm
			alg->setDistanceThresholdAndGetPoints(unifyCirclePoints, planeparam, distance, 0, 0.2, out_circlePoints);
			cout << "�������������" << out_circlePoints.size() << endl;
			ofstream outfile;
			fileName = vp->engineNumFileName + "/outputFile/stepFive/firstPart/engineCirclePointsThroughDistanceThreshold/" + time
				+ " engineCirclePointsThroughDistanceThreshold.txt";
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

			fileName = vp->engineNumFileName + "/outputFile/stepFive/firstPart/engineFilterBorderPoints/" + time
				+ " engineFilterBorderPoints.txt";
			outfile.open(fileName.toStdString());
			for (int i = 0; i < filterBorderPoints.size(); i++){
				outfile << filterBorderPoints[i].x << " " << filterBorderPoints[i].y << " " << filterBorderPoints[i].z << endl;
			}
			outfile.close();

			//����任
			filterBorderPoints = vp->MeasureToRobotTool(filterBorderPoints);
			centerResult = vp->MeasureToRobotTool(centerResult);
			planeparam = vp->MeasureToRobotTool(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//����任
			filterBorderPoints = vp->RobotToolToBase(filterBorderPoints);
			centerResult = vp->RobotToolToBase(centerResult);
			planeparam = vp->RobotToolToBase(planeparam);
			cout << "����任��" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//���洦����Ϣ
			QString cirlceStatusStr;
			if (abs(averageDistance - R) < 0.1){
				centerResult.status = true;
				cirlceStatusStr = "true";
			}
			else{
				centerResult.status = false;
				cirlceStatusStr = "false";
			}
			time = vp->ReturnTime();
			fileName = vp->engineNumFileName + "/outputFile/stepFive/firstPart/engineParam.csv";//д���ļ���Ŀ¼
			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time << "," << num << "," << centerResult.center.x << "," << centerResult.center.y << "," << centerResult.center.z << "," << cirlceStatusStr << ","
				<< planeparam.A << "," << planeparam.B << "," << planeparam.C << ","
				<< averageDistance << "\n";
			file.close();
			//����������

			fileName = vp->engineNumFileName + "/outputFile/stepFive/firstPart/engineFinalBorderPoints/" + time
				+ " engineFinalBorderPoints.txt";
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
		else
		{
			CoreAlgorithm::StereoCircle centerResult;
			centerResult.status = false;
			bool planeparamStatus = true;
			time = vp->ReturnTime();
			fileName = vp->engineNumFileName + "/outputFile/stepFive/firstPart/engineParam.csv";//д���ļ���Ŀ¼
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
		fileName = vp->engineNumFileName + "/outputFile/stepFive/firstPart/engineParam.csv";//д���ļ���Ŀ¼
		QFile file(fileName);
		file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
		QTextStream out(&file);
		out << time << "," << num << "," << "null" /*Բ��X*/ << "," << "null" /*Բ��Y*/ << "," << "null"/*Բ��Z*/ << "," << "false"/*Բ�����״̬*/ << ","
			<< "null"/*��ʸA*/ << "," << "null"/*��ʸB*/ << "," << "null"/*��ʸC*/ << "," << "null" /*��Ե�㵽Բ�ĵ�ƽ������*/ << "\n";
		file.close();
		emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
	}
}
//H502 ���ǰ�����仨�����ƽ�淨ʸ��Բ��
void ProcessThread::stepFiveRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond)
{
	//�����ͷ����ϵ��Y������
	double RightIncrementFirst;
	double RightIncrementSecond;
	//RightIncrementFirst = ui.RightRunFirst_StepFive->text().toDouble();
	//RightIncrementSecond = ui.RightRunFirst_StepFive->text().toDouble() + ui.RightCameraScan_StepFive->text().toDouble()
	//	+ ui.RightRunSecond_StepFive->text().toDouble();
	vp->calculateMoveDistance(num, RightIncrementFirst, RightIncrementSecond);
#ifndef SETUP
	RightIncrementFirst = rightRunDis[0];
	RightIncrementSecond = rightRunDis[1];
#endif//SETUP
	QString runtime = vp->ReturnTime();
	QString runfileName = vp->engineNumFileName + "/outputFile/stepFive/GratingData.csv";//д���ļ���Ŀ¼
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
	circleFirstImage = alg->getfrontDriveBoxSplineEdgeImage(inputImageFirst, status_circleFirst);
	circleSecondImage = alg->getfrontDriveBoxSplineEdgeImage(inputImageSecond, status_circleSecond);
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
	fileName = vp->engineNumFileName + "/outputFile/stepFive/firstPart/frontDriveBoxUnifyCirclePoints/" + time + " frontDriveBoxUnifyCirclePoints.txt";
	QFile file(fileName);
	file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
	QTextStream out(&file);
	for (int i = 0; i < unifyCirclePoints.size(); i++){
		out << unifyCirclePoints[i].x << " " << unifyCirclePoints[i].y << " " << unifyCirclePoints[i].z << endl;
	}
	file.close();
	if (unifyPlanePoints.size() != 0)
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
		if (unifyPlanePoints.size() != 0)
		{
			//�����Ե�㵽��Ͽռ�ƽ��ľ��룬�����þ�����ֵ��ȡ������ֵ�ĵ�
			//�������������
			Vector<Point3f> out_circlePoints;
			//�����㵽ƽ����뼯��
			Vector<double> distance;
			//��������ֵΪmm����������ֵΪmm
			alg->setDistanceThresholdAndGetPoints(unifyCirclePoints, planeparam, distance, 0, 0.2, out_circlePoints);
			cout << "�������������" << out_circlePoints.size() << endl;
			ofstream outfile;
			fileName = vp->engineNumFileName + "/outputFile/stepFive/firstPart/frontDriveBoxCirclePointsThroughDistanceThreshold/" + time
				+ " frontDriveBoxCirclePointsThroughDistanceThreshold.txt";
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
			filterBorderPoints = alg->filterBorderPointsOnDistanceThrehold(subpoints, distances, R, 0.2);
			cout << "����ǰ���� = " << subpoints.size() << endl;
			cout << "���˺���� = " << filterBorderPoints.size() << endl;
			centerResult = alg->fitSteroCircle(filterBorderPoints, planeparam, R);
			cout << "�ռ�Բ�����꣺" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			averageDistance = alg->CalculateBorderPointsToCenterAverageDistance(filterBorderPoints, centerResult);
			cout << endl << "��Ե�㵽�ռ�Բ�ĵ�ƽ������Ϊ�� " << averageDistance << endl << endl;

			fileName = vp->engineNumFileName + "/outputFile/stepFive/firstPart/frontDriveBoxFilterBorderPoints/" + time
				+ " frontDriveBoxFilterBorderPoints.txt";
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
			//���洦����Ϣ
			QString cirlceStatusStr;
			if (abs(averageDistance - R) < 0.1){
				centerResult.status = true;
				cirlceStatusStr = "true";
			}
			else{
				centerResult.status = false;
				cirlceStatusStr = "false";
			}
			time = vp->ReturnTime();
			fileName = vp->engineNumFileName + "/outputFile/stepFive/firstPart/frontDriveBoxParam.csv";//д���ļ���Ŀ¼
			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time << "," << num << "," << centerResult.center.x << "," << centerResult.center.y << "," << centerResult.center.z << "," << cirlceStatusStr << ","
				<< planeparam.A << "," << planeparam.B << "," << planeparam.C << ","
				<< averageDistance << "\n";
			file.close();
			//����������

			fileName = vp->engineNumFileName + "/outputFile/stepFive/firstPart/frontDriveBoxFinalBorderPoints/" + time
				+ " frontDriveBoxFinalBorderPoints.txt";
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
		else
		{
			CoreAlgorithm::StereoCircle centerResult;
			centerResult.status = false;
			bool planeparamStatus = true;
			time = vp->ReturnTime();
			fileName = vp->engineNumFileName + "/outputFile/stepFive/firstPart/frontDriveBoxParam.csv";//д���ļ���Ŀ¼
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
		fileName = vp->engineNumFileName + "/outputFile/stepFive/firstPart/frontDriveBoxParam.csv";//д���ļ���Ŀ¼
		QFile file(fileName);
		file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
		QTextStream out(&file);
		out << time << "," << num << "," << "null" /*Բ��X*/ << "," << "null" /*Բ��Y*/ << "," << "null"/*Բ��Z*/ << "," << "false"/*Բ�����״̬*/ << ","
			<< "null"/*��ʸA*/ << "," << "null"/*��ʸB*/ << "," << "null"/*��ʸC*/ << "," << "null" /*��Ե�㵽Բ�ĵ�ƽ������*/ << "\n";
		file.close();
		emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
	}
}

void ProcessThread::initialization()
{
#ifndef SETUP
	double startPosition;
	double endPosition;
	cap->initializationCamera();
	motion->initializationMotion();
	startPosition = motion->GetCurrentPositon();
	//��ʼʱ�ɼ�һ��
	/**
	*������Ϊ���ֳ���������������һ��
	*�ɼ����������̫׼ȷ
	*/
	//motion->leadRailMotion(50);
	HObject IMG;
	//��ȡ�����˶��ٶ�
	double speed;
	speed = cap->RateToSpeed(cap->hv_RateValue);
	//�������쿪ʼ�˶�
	//sendPLC();
	motion->setLeadRailVelocity(speed);
	motion->setLeadRailMotionDistance(50);
	IMG = cap->captureImage();
	motion->CheckRun();
	Sleep(100);
	endPosition = motion->GetCurrentPositon();
	motion->reset(startPosition, endPosition);
	motion->CheckRun();
#endif
	

}

void ProcessThread::motionActionSlot()
{
	motion->show();
}

void ProcessThread::cameraActionSlot()
{
	cap->show();
}

//stepCapture * ProcessThread::getStepCapture(stepName name)
//{
//	static stepCapture sc[2];
//	switch (name)
//	{
//	case h_stepOneFirstPart:
//		sc[0] = left_hOneFirstP;
//		sc[1] = right_hOneFirstP;
//		break;
//	case h_stepOneSecondPart:
//		sc[0] = left_hOneSecondP;
//		sc[1] = right_hOneSecondP;
//		break;
//	case h_stepOneThirdPart:
//		sc[0] = left_hOneThirdP;
//		sc[1] = right_hOneThirdP;
//		break;
//
//	case h_stepTwoFirstPart:
//		sc[0] = left_hTwoFirstP;
//		sc[1] = right_hTwoFirstP;
//		break;
//	case h_stepTwoSecondPart:
//		sc[0] = left_hTwoSecondP;
//		sc[1] = right_hTwoSecondP;
//		break;
//	case h_stepTwoThirdPart:
//		sc[0] = left_hTwoThirdP;
//		sc[1] = right_hTwoThirdP;
//		break;
//	case h_stepTwoForthPart:
//		sc[0] = left_hTwoForthP;
//		sc[1] = right_hTwoForthP;
//		break;
//
//	case h_stepThreeFirstPart:
//		sc[0] = left_hThreeFirstP;
//		sc[1] = right_hThreeFirstP;
//		break;
//	case h_stepThreeSecondPart:
//		sc[0] = left_hThreeSecondP;
//		sc[1] = right_hThreeSecondP;
//		break;
//	case h_stepThreeThirdPart:
//		sc[0] = left_hThreeThirdP;
//		sc[1] = right_hThreeThirdP;
//		break;
//	case h_stepThreeForthPart:
//		sc[0] = left_hThreeForthP;
//		sc[1] = right_hThreeForthP;
//		break;
//
//	case h_stepFourFirstPart:
//		sc[0] = left_hFourFirstP;
//		sc[1] = right_hFourFirstP;
//		break;
//	case h_stepFourSecondPart:
//		sc[0] = left_hFourSecondP;
//		sc[1] = right_hFourSecondP;
//		break;
//	case h_stepFourThirdPart:
//		sc[0] = left_hFourThirdP;
//		sc[1] = right_hFourThirdP;
//		break;
//	case h_stepFourForthPart:
//		sc[0] = left_hFourForthP;
//		sc[1] = right_hFourForthP;
//		break;
//
//	case h_stepFiveFirstPart:
//		sc[0] = left_hFiveFirstP;
//		sc[1] = right_hFiveFirstP;
//		break;
//	case h_stepFiveSecondPart:
//		sc[0] = left_hFiveSecondP;
//		sc[1] = right_hFiveSecondP;
//		break;
//	case h_stepFiveThirdPart:
//		sc[0] = left_hFiveThirdP;
//		sc[1] = right_hFiveThirdP;
//		break;
//	case h_stepFiveForthPart:
//		sc[0] = left_hFiveForthP;
//		sc[1] = right_hFiveForthP;
//		break;
//
//		//�ָ�
//	case v_stepOneFirstPart:
//		sc[0] = left_vOneFP;
//		sc[1] = right_vOneFP;
//		break;
//	case v_stepOneSecondPart:
//		sc[0] = left_vOneSP;
//		sc[1] = right_vOneSP;
//		break;
//	case v_stepTwoFirstPart:
//		sc[0] = left_vTwoFP;
//		sc[1] = right_vTwoFP;
//		break;
//	case v_stepTwoSecondPart:
//		sc[0] = left_vTwoSP;
//		sc[1] = right_vTwoSP;
//		break;
//	case v_stepThreeFirstPart:
//		sc[0] = left_vThreeFP;
//		sc[1] = right_vThreeFP;
//		break;
//	case v_stepThreeSecondPart:
//		sc[0] = left_vThreeSP;
//		sc[1] = right_vThreeSP;
//		break;
//	default:
//		break;
//	}
//	return sc;
//}

void ProcessThread::v_stepThreeFirstLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond)
{

}

void ProcessThread::v_stepThreeFirstRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond)
{

}

void ProcessThread::v_stepThreeSecondLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond)
{

}

void ProcessThread::v_stepThreeSecondRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond)
{

} 

HalconCpp::HObject ProcessThread::getPlaneImage(HObject &inputImage, bool& status)
{
	// Local iconic variables
	HObject  ho_Regions, ho_ImageReduced;
	HObject  ho_ImageScale, ho_Region, ho_ImageX, ho_ImageY;
	HObject  ho_ImageZ_, ho_ImageZ;
	HObject  ho_ROI_0, ho_ROI_1;
	HObject  ho_RegionUnion, ho_ROI_3, ho_ROI_4, ho_ROI_5, ho_ROI_6;
	HObject  ho_ROI_7, ho_RegionIntersection;
	// Local control variables
	HTuple  hv_WindowHandle, hv_xResolution, hv_yResolution;
	HTuple  hv_ZoomScale, hv_CalibFile, hv_Homography, hv_gFactor;
	HTuple  hv_NumCoefficients, hv_Tinv1, hv_Tinv2, hv_savefile;
	HTuple  hv_Width, hv_Height, hv_Rows, hv_Columns, hv_Grayval;
	HTuple  hv_X, hv_Y_, hv_Z, hv_Qx, hv_Qy, hv_Qw, hv_X_, hv_Z_;
	HTuple  hv_HomographyInvert, hv_Ox, hv_Oz, hv_Ow, hv_X_NEW;
	HTuple  hv_Z_NEW, hv_ObjectModel3D, hv_ObjectModel3D_NEW;
	HTuple  hv_PoseVis, hv_i, hv_xcd, hv_ycd, hv_zcd, hv_num;
	HTuple  hv_Seconds1, hv_FileHandle, hv_Seconds2, hv_j, hv_Time;
	HTuple  hv_Area, hv_Row, hv_Area2, hv_Row2, hv_Column2;
	HTuple  hv_Column, hv_status, hv_Exception;
	//***********************************************************************************
	try
	{
		//ͼ����
		GetImageSize(inputImage, &hv_Width, &hv_Height);
		Threshold(inputImage, &ho_Regions, 1, 65535);
		AreaCenter(ho_Regions, &hv_Area, &hv_Row, &hv_Column);

		GenCircle(&ho_ROI_0, hv_Row, hv_Column, 200);
		GenCircle(&ho_ROI_1, hv_Row / 2, hv_Column, 200);
		Union2(ho_ROI_0, ho_ROI_1, &ho_RegionUnion);
		GenCircle(&ho_ROI_3, hv_Row + 400, hv_Column, 200);
		Union2(ho_RegionUnion, ho_ROI_3, &ho_RegionUnion);
		GenCircle(&ho_ROI_4, hv_Row, hv_Column / 2, 200);
		Union2(ho_RegionUnion, ho_ROI_4, &ho_RegionUnion);
		GenCircle(&ho_ROI_5, hv_Row, hv_Column + ((hv_Width - hv_Column) / 2), 200);
		Union2(ho_RegionUnion, ho_ROI_5, &ho_RegionUnion);
		GenCircle(&ho_ROI_6, hv_Row + ((hv_Height - hv_Row) / 2), hv_Column + ((hv_Width - hv_Column) / 4),
			200);
		Union2(ho_RegionUnion, ho_ROI_6, &ho_RegionUnion);
		GenCircle(&ho_ROI_7, hv_Row + ((hv_Height - hv_Row) / 2), hv_Column - ((hv_Width - hv_Column) / 4),
			200);
		Union2(ho_RegionUnion, ho_ROI_7, &ho_RegionUnion);
		Intersection(ho_Regions, ho_RegionUnion, &ho_RegionIntersection);

		AreaCenter(ho_RegionIntersection, &hv_Area2, &hv_Row2, &hv_Column2);
		if (0 != (hv_Area2 == 0))
		{
			status = false;
		}
		else{
			status = true;
		}



		ReduceDomain(inputImage, ho_RegionIntersection, &ho_ImageReduced);
	}
	// catch (Exception) 
	catch (HalconCpp::HException &HDevExpDefaultException)
	{
		HDevExpDefaultException.ToHTuple(&hv_Exception);
		status = false;
	}

	return ho_ImageReduced;
}





