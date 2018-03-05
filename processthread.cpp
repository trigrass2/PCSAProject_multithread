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
	//工位信息
	connect(this, &ProcessThread::sendStepNum, vp, &VisualProcessing::receiveStepNum);
	connect(vp, &VisualProcessing::sendStepParam, this, &ProcessThread::receiveStepParam);
	connect(this, &ProcessThread::sendProcessingInformation, vp, &VisualProcessing::receiveInformation);
	connect(vp, &VisualProcessing::sendProcessingResult, this, &ProcessThread::receiveProcessingResult);

	qRegisterMetaType<HObject>("HObject");
	qRegisterMetaType<CoreAlgorithm::PlaneNormal>("CoreAlgorithm::PlaneNormal");
	qRegisterMetaType<QString>("QString");//传送图片，保存名
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
	//统计程序运行时间
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
	sendStatusUpdate(QStringLiteral("左侧头扫描完毕"), 0);
	Sleep(100);
#ifndef SETUP
	//转台旋转180度.
	motion->platformLtoRRotate();
#endif
	Sleep(500);
	RightScannerRun(102, transmissionCaseCircleFirst_cloudPoints, transmissionCaseCenterResultFirst);
	sendStatusUpdate(QStringLiteral("右侧头扫描完毕"), 0);
#ifndef SETUP
	motion->CheckRun();
	Sleep(100);
	motion->platformRtoLRotate();
#endif // !SETUP
	cout << "圆心: " << endl;
	/*cout << leftCenterResult.center.x << " " << leftCenterResult.center.y << " " << leftCenterResult.center.z << endl;*/
	cout << transmissionCaseCenterResultFirst.center.x
		<< " " << transmissionCaseCenterResultFirst.center.y
		<< " " << transmissionCaseCenterResultFirst.center.z << endl;
	//显示边缘点云及圆心
#ifndef PROGRESS

#endif//！PROGRESS
	sendStatusUpdate(QStringLiteral("工位1第一部分运行完毕"), 0);
	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	cout << "此程序的运行时间为" << totaltime << "秒！" << endl;

	//结束线程
	threadStatus = false;

}

void ProcessThread::LeftScannerRun(int num, pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult)
{
	//统计程序运行时间
	clock_t start, finish;
	double totaltime;
	double startPosition;
	double endPosition;
	double position;
	//***声明变量**************************************************************//

	//获取工步相关参数
	emit sendStepNum(num);
	//获取导轨当前位置
#ifndef SETUP
	startPosition = motion->GetCurrentPositon();
#endif //SETUP

	//声明深度图
	HObject inputImageFirst, inputImageSecond;
	//声明变量
	double speed;
	Vector<Point3f> points;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
	start = clock();
#ifndef SETUP
	//光删尺计数清零
	serial->clearGratingData();
	Sleep(100);
	//设置导轨运动速度
	motion->setLeadRailVelocity(CameraNoScanSpeed);
	//运动到第一次采集位置
	motion->setLeadRailMotionDistance(CameraFirstNoScanDistance);
	motion->CheckRun();
	cout << "开始采集位置：" << endl;
	position = motion->GetCurrentPositon();
	Sleep(500);
	//读取光删尺反馈导轨运动距离
	serial->readGratingData(leftRunDis);
	Sleep(500);
	//获取导轨运动速度
	speed = cap->RateToSpeed(cap->hv_RateValue);
	//设置导轨运动速度
	motion->setLeadRailVelocity(speed);
	//获取图像
	GrabImageStart(cap->hv_AcqHandle, -1);
	//触发相机
	cap->sendPLC();
	//启动导轨开始运动, 移动距离为50mm
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
	//显示并且保存图片
	emit sendImageAndSave(inputImageFirst, 101, "LeftFirst");
	////显示图片
	//displayImage(inputImageFirst, 101);
	////保存图片
	//SaveImage(inputImageFirst, "LeftFirst");

#ifndef SETUP
	if (num == 203 || num == 303 || num == 2030){
		HObject  ho_EmptyRegion;
		GenEmptyRegion(&ho_EmptyRegion);
		RegionToBin(ho_EmptyRegion, &inputImageSecond, 255, 0, 4096, 2000);
	}
	else{
		motion->CheckRun();
		//设置导轨运动速度
		motion->setLeadRailVelocity(CameraNoScanSpeed);
		//运动到第二次采集位置
		motion->setLeadRailMotionDistance(CameraSecondNoScanDistance);
		motion->CheckRun();
		cout << "第二次采集位置：" << endl;
		position = motion->GetCurrentPositon();
		Sleep(500);
		//读取光删尺反馈导轨运动距离
		serial->readGratingData(leftRunDis);
		Sleep(500);
		//启动导轨开始运动
		motion->setLeadRailVelocity(speed);
		//启动导轨开始运动, 移动距离为50mm

		//获取图像
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
	////显示图片
	//displayImage(inputImageSecond, 102);
	////保存图片
	//SaveImage(inputImageSecond, "LeftSecond");

#ifndef SETUP
	motion->CheckRun();
	Sleep(100);
	cout << "测试" << endl;
	cout << "运动结束位置：" << endl;
	position = motion->GetCurrentPositon();
	Sleep(500);
	//获取采集结束位置
	endPosition = motion->GetCurrentPositon();
	//导轨复位
	motion->reset(startPosition, endPosition);
	cout << "测试" << endl;
	//发送数据
	emit sendLeftGratingData(leftRunDis);
	//数据清空，为下次储存做准备
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
	//清空变量
	points.clear();

	cout << "此程序的运行时间为" << totaltime << "秒！" << endl;
}

void ProcessThread::RightScannerRun(int num, pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult)
{
	//统计程序运行时间
	clock_t start, finish;
	double totaltime;
	double startPosition;
	double endPosition;
	double position;
	//***声明变量**************************************************************//
	//获取导轨当前位置
#ifndef SETUP
	startPosition = motion->GetCurrentPositon();
#endif //SETUP
	//获取工步相关参数
	emit sendStepNum(num);
	//声明深度图
	HObject inputImageFirst, inputImageSecond;
	//声明变量
	double speed;
	Vector<Point3f> points;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
	start = clock();
#ifndef SETUP
	serial->clearGratingData();
	Sleep(100);
	motion->setLeadRailVelocity(CameraNoScanSpeed);
	//运动到第一次采集位置
	motion->setLeadRailMotionDistance(CameraFirstNoScanDistance);
	motion->CheckRun();
	cout << "开始采集位置：" << endl;
	position = motion->GetCurrentPositon();
	Sleep(500);
	//读取光删尺反馈导轨运动距离
	serial->readGratingData(rightRunDis);
	Sleep(500);
	//获取导轨运动速度
	speed = cap->RateToSpeed(cap->hv_RateValue);
	//设置导轨运动速度
	motion->setLeadRailVelocity(speed);
	//获取图像
	GrabImageStart(cap->hv_AcqHandle, -1);
	cap->sendPLC();
	//启动导轨开始运动, 移动距离为50mmCameraScanDistance
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
	////显示图片
	//displayImage(inputImageFirst, 201);
	////保存图片
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
		//设置导轨运动速度
		motion->setLeadRailVelocity(CameraNoScanSpeed);
		//运动到第二次采集位置
		motion->setLeadRailMotionDistance(CameraSecondNoScanDistance);
		motion->CheckRun();
		cout << "第二次采集位置：" << endl;
		position = motion->GetCurrentPositon();
		Sleep(500);
		//读取光删尺反馈导轨运动距离
		serial->readGratingData(rightRunDis);
		Sleep(500);
		//启动导轨开始运动
		motion->setLeadRailVelocity(speed);
		//启动导轨开始运动, 移动距离为50mm

		//获取图像
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

	////显示图片
	//displayImage(inputImageSecond, 202);
	////保存图片
	//SaveImage(inputImageSecond, "RightSecond");
#ifndef SETUP
	motion->CheckRun();
	Sleep(100);
	cout << "测试" << endl;
	cout << "运动结束位置：" << endl;
	position = motion->GetCurrentPositon();
	Sleep(500);
	//获取采集结束位置
	endPosition = motion->GetCurrentPositon();
	//导轨复位
	motion->reset(startPosition, endPosition);
	cout << "测试" << endl;
	//发送数据
	emit sendRightGratingData(rightRunDis);
	//数据清空，为下次储存做准备
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
	//清空变量
	points.clear();

	cout << "此程序的运行时间为" << totaltime << "秒！" << endl;
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
	cout << "传输信息：" << endl;
	cout << "	测头非扫描时运动速度" << CameraNoScanSpeed << endl;
	cout << "	测头扫描时运动距离" << CameraScanDistance << endl;
	cout << "	测头运动到第一测量位置的运动距离" << CameraFirstNoScanDistance << endl;
	cout << "	测头运动到第二测量位置的运动距离" << CameraSecondNoScanDistance << endl;
}

void ProcessThread::receiveImageNum(int num)
{
	imageNum = num;
}

void ProcessThread::receiveProcessingResult(bool planeparamStatus/*平面法矢处理状态*/, CoreAlgorithm::PlaneNormal planeparam/*平面法矢*/, CoreAlgorithm::StereoCircle centerResult/*空间圆心*/, pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr/*边缘点云 PCL格式*/, int num/*工步号*/)
{
	emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
}

void ProcessThread::stepOneFirstLeftProcessing(int num, HObject inputImageFirst, HObject inputImageSecond)
{
	//定义测头坐标系下Y轴增量
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
	QString runfileName = vp->engineNumFileName + "/outputFile/StepOne/GratingData.csv";//写入文件的目录
	QFile runfile(runfileName);
	runfile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
	QTextStream runout(&runfile);
	runout << runtime << "," << "left" << "," << LeftIncrementFirst << "," << LeftIncrementSecond << "\n";
	runfile.close();

	//声明平面处理图片
	HObject planeFirstImage, planeSecondImage;
	//声明边缘处理图片
	//HObject circleFirstImage, circleSecondImage;
	//声明平面点集
	Vector<Point3f> planePointsFirst, planePointsSecond;
	//声明统一坐标系下平面点集
	Vector<Point3f> unifyPlanePoints;
	//声明边缘点集
	//Vector<Point3f> circlePointsFirst, circlePointsSecond;
	//声明统一坐标系下边缘点集
	//Vector<Point3f> unifyCirclePoints;
	//声明图像处理结果 true or false
	bool status_planeFirst, status_planeSecond;
	//bool status_circleFirst, status_circleSecond;
	//获取平面处理图片
	planeFirstImage = alg->getPlaneImage(inputImageFirst, status_planeFirst);
	planeSecondImage = alg->getPlaneImage(inputImageSecond, status_planeSecond);
	//获取边缘处理图片
	//circleFirstImage = alg->getFlangeEdgeImage(inputImageFirst, status_circleFirst);
	//circleSecondImage = alg->getFlangeEdgeImage(inputImageSecond, status_circleSecond);
	//获取平面点集
	planePointsFirst = alg->depthMapCalibration(planeFirstImage, status_planeFirst);
	planePointsSecond = alg->depthMapCalibration(planeSecondImage, status_planeSecond);
	cout << "1平面点集数量：" << planePointsFirst.size() << endl;
	cout << "2平面点集数量：" << planePointsSecond.size() << endl;
	//获取边缘点集
	//circlePointsFirst = alg->depthMapCalibration(circleFirstImage, status_circleFirst);
	//circlePointsSecond = alg->depthMapCalibration(circleSecondImage, status_circleSecond);
	//cout << "1边缘点集数量：" << circlePointsFirst.size() << endl;
	//cout << "2边缘点集数量：" << circlePointsSecond.size() << endl;
	//采集点，统一坐标系
	for (int i = 0; i < planePointsFirst.size(); i++){
		unifyPlanePoints.push_back(Point3f(planePointsFirst[i].x, planePointsFirst[i].y + LeftIncrementFirst, planePointsFirst[i].z));
	}

	for (int i = 0; i < planePointsSecond.size(); i++){
		unifyPlanePoints.push_back(Point3f(planePointsSecond[i].x, planePointsSecond[i].y + LeftIncrementSecond, planePointsSecond[i].z));
	}
	if (unifyPlanePoints.size() != 0)
	{
		//声明平面点云（PCL）
		pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
		//声明边缘点云（PCL）
		//pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		//平面点集转化为PCL点云
		alg->threeDimensionToPCLPoints(unifyPlanePoints, plane_cloudPoints);
		//声明空间平面法矢数据结构
		CoreAlgorithm::PlaneNormal planeparam;
		//获取平面法矢
		planeparam = alg->getPlaneNormal(plane_cloudPoints);
		

		//坐标变换
		//filterBorderPoints = MeasureToRobot(filterBorderPoints);
		//	centerResult = MeasureToRobot(centerResult);
		planeparam = vp->MeasureToRobotTool(planeparam);
		planeparam = vp->RobotToolToBase(planeparam);
		cout << "坐标变换：" << endl;
		//cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
		cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;

		QString time = vp->ReturnTime();
		QString fileName = vp->engineNumFileName + "/outputFile/StepOne/firstPart/engineParam.csv";//写入文件的目录
		QFile file(fileName);
		file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
		QTextStream out(&file);
		out << time << "," << num << "," << planeparam.A << "," << planeparam.B << "," << planeparam.C << "\n";
		file.close();
		bool planeparamStatus = true;
		//声明空间球心
		CoreAlgorithm::StereoCircle centerResult;
		//声明边缘点云（PCL）
		pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);

	}
	else
	{
		bool planeparamStatus = false;
		//声明空间平面法矢数据结构
		CoreAlgorithm::PlaneNormal planeparam;
		QString time = vp->ReturnTime();
		QString fileName = vp->engineNumFileName + "/outputFile/StepOne/firstPart/engineNormal.csv";//写入文件的目录
		QFile file(fileName);
		file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
		QTextStream out(&file);
		out << time << "," << num << "," << "null" << "," << "null" << "," << "null" << "\n";
		file.close();
		//声明空间球心
		CoreAlgorithm::StereoCircle centerResult;
		//声明边缘点云（PCL）
		pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
	}
}

void ProcessThread::stepOneSecondRightProcessing(int num, HObject inputImageFirst, HObject inputImageSecond)
{
	//定义测头坐标系下Y轴增量
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
	QString runfileName = vp->engineNumFileName + "/outputFile/stepOne/GratingData.csv";//写入文件的目录
	QFile runfile(runfileName);
	runfile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
	QTextStream runout(&runfile);
	runout << runtime << "," << "right" << "," << RightIncrementFirst << "," << RightIncrementSecond << "\n";
	runfile.close();
	//声明平面处理图片
	HObject planeFirstImage, planeSecondImage;
	//声明边缘处理图片
	HObject circleFirstImage, circleSecondImage;
	//声明平面点集
	Vector<Point3f> planePointsFirst, planePointsSecond;
	//声明统一坐标系下平面点集
	Vector<Point3f> unifyPlanePoints;
	//声明边缘点集
	Vector<Point3f> circlePointsFirst, circlePointsSecond;
	//声明统一坐标系下边缘点集
	Vector<Point3f> unifyCirclePoints;
	//声明图像处理结果 true or false
	bool status_planeFirst, status_planeSecond;
	bool status_circleFirst, status_circleSecond;
	//获取平面处理图片
	planeFirstImage = alg->getPlaneImage(inputImageFirst, status_planeFirst);
	planeSecondImage = alg->getPlaneImage(inputImageSecond, status_planeSecond);
	//获取边缘处理图片
	if (num == 102 || num == 104){
		circleFirstImage = alg->getFlangeEdgeImage(inputImageFirst, status_circleFirst);
		circleSecondImage = alg->getFlangeEdgeImage(inputImageSecond, status_circleSecond);
	}//横置发动机
	if (num == 1020 || num == 1040){
		circleFirstImage = alg->getFlangeEdgeImage(inputImageFirst, status_circleFirst);
		circleSecondImage = alg->getFlangeEdgeImage(inputImageSecond, status_circleSecond);
	}//纵置发动机

	//获取平面点集
	planePointsFirst = alg->depthMapCalibration(planeFirstImage, status_planeFirst);
	planePointsSecond = alg->depthMapCalibration(planeSecondImage, status_planeSecond);
	cout << "1平面点集数量：" << planePointsFirst.size() << endl;
	cout << "2平面点集数量：" << planePointsSecond.size() << endl;
	//获取边缘点集
	circlePointsFirst = alg->depthMapCalibration(circleFirstImage, status_circleFirst);
	circlePointsSecond = alg->depthMapCalibration(circleSecondImage, status_circleSecond);
	cout << "1边缘点集数量：" << circlePointsFirst.size() << endl;
	cout << "2边缘点集数量：" << circlePointsSecond.size() << endl;
	//采集点，统一坐标系
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
	cout << "统一坐标后平面点集数量：" << unifyPlanePoints.size() << endl;
	cout << "统一坐标后边缘点集数量：" << unifyCirclePoints.size() << endl;

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
	if (unifyPlanePoints.size() != 0)//先检测是否可以进行平面拟合
	{
		//声明平面点云（PCL）
		pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
		//声明边缘点云（PCL）
		pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		//平面点集转化为PCL点云
		alg->threeDimensionToPCLPoints(unifyPlanePoints, plane_cloudPoints);
		//声明空间平面法矢数据结构
		CoreAlgorithm::PlaneNormal planeparam;
		//获取平面法矢
		planeparam = alg->getPlaneNormal(plane_cloudPoints);
		if (unifyCirclePoints.size() != 0)//检测是否可以进行圆心拟合
		{
			//计算边缘点到拟合空间平面的距离，并设置距离阈值获取符合阈值的点
			//声明函数输出点
			Vector<Point3f> out_circlePoints;
			//声明点到平面距离集合
			Vector<double> distance;
			//设置下阈值为mm，设置上阈值为mm
			alg->setDistanceThresholdAndGetPoints(unifyCirclePoints, planeparam, distance, 0, 0.7, out_circlePoints);
			cout << "输出点云数量：" << out_circlePoints.size() << endl;
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

			//声明空间球心

			CoreAlgorithm::StereoCircle centerResult;
			//进行空间圆拟合
			//半径约束
			double R;
			vp->getRestrictRadius(num, R);
			Vector<Point3f> subpoints;
			//计算边缘点到拟合平面的投影点
			subpoints = alg->ComputedEdgeProjectionPoints(out_circlePoints, planeparam);
			centerResult = alg->fitSteroCircle(subpoints, planeparam, R);
			double averageDistance;
			averageDistance = alg->CalculateBorderPointsToCenterAverageDistance(subpoints, centerResult);
			cout << endl << "边缘点到空间圆心的平均距离为： " << averageDistance << endl << endl;
			cout << "空间圆心坐标：" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			Vector<double>distances;
			alg->CalculateBorderPointsToCenterDistance(subpoints, centerResult, distances);
			Vector<Point3f>filterBorderPoints;
			filterBorderPoints = alg->filterBorderPointsOnDistanceThrehold(subpoints, distances, R, 0.05);
			cout << "过滤前点数 = " << subpoints.size() << endl;
			cout << "过滤后点数 = " << filterBorderPoints.size() << endl;
			centerResult = alg->fitSteroCircle(filterBorderPoints, planeparam, R);
			cout << "空间圆心坐标：" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			averageDistance = alg->CalculateBorderPointsToCenterAverageDistance(filterBorderPoints, centerResult);
			cout << endl << "边缘点到空间圆心的平均距离为： " << averageDistance << endl << endl;
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

			//坐标变换
			//右测头坐标系到左侧头坐标系
			filterBorderPoints = vp->RightToLeft(filterBorderPoints);
			centerResult = vp->RightToLeft(centerResult);
			planeparam = vp->RightToLeft(planeparam);
			cout << "坐标变换：" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//测头坐标系到机器人工具坐标系
			filterBorderPoints = vp->MeasureToRobotTool(filterBorderPoints);
			centerResult = vp->MeasureToRobotTool(centerResult);
			planeparam = vp->MeasureToRobotTool(planeparam);
			cout << "坐标变换：" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//机器人工具坐标系到机器人基座坐标系
			filterBorderPoints = vp->RobotToolToBase(filterBorderPoints);
			centerResult = vp->RobotToolToBase(centerResult);
			planeparam = vp->RobotToolToBase(planeparam);
			cout << "坐标变换：" << endl;
			cout << "centerResult :" << centerResult.center.x << " " << centerResult.center.y << " " << centerResult.center.z << endl;
			cout << "planeparam :" << planeparam.A << " " << planeparam.B << " " << planeparam.C << endl;
			//保存处理结果
			time = vp->ReturnTime();
			if (num == 102){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/frontDriveBoxParam.csv";//写入文件的目录
			}
			if (num == 1020){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/syntheticalDriveBoxParam.csv";//写入文件的目录
			}
			if (num == 104){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/frontDriveBoxParam.csv";//写入文件的目录
			}
			if (num == 1040){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/syntheticalDriveBoxParam.csv";//写入文件的目录
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
			//保存最后处理结果
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
			//边缘点集转化为PCL点云
			alg->threeDimensionToPCLPoints(filterBorderPoints, borderCloud_ptr);
			emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
		}
		else{
			CoreAlgorithm::StereoCircle centerResult;
			centerResult.status = false;
			bool planeparamStatus = true;
			if (num == 102){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/frontDriveBoxParam.csv";//写入文件的目录
			}
			if (num == 1020){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/syntheticalDriveBoxParam.csv";//写入文件的目录
			}
			if (num == 104){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/frontDriveBoxParam.csv";//写入文件的目录
			}
			if (num == 1040){
				fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/syntheticalDriveBoxParam.csv";//写入文件的目录
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
		//声明空间平面法矢数据结构
		CoreAlgorithm::PlaneNormal planeparam;
		//声明空间球心
		CoreAlgorithm::StereoCircle centerResult;
		centerResult.status = false;
		//声明边缘点云（PCL）
		pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		time = vp->ReturnTime();
		if (num == 102){
			fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/frontDriveBoxParam.csv";//写入文件的目录
		}
		if (num == 1020){
			fileName = vp->engineNumFileName + "/outputFile/stepOne/firstPart/syntheticalDriveBoxParam.csv";//写入文件的目录
		}
		if (num == 104){
			fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/frontDriveBoxParam.csv";//写入文件的目录
		}
		if (num == 1040){
			fileName = vp->engineNumFileName + "/outputFile/stepOne/secondPart/syntheticalDriveBoxParam.csv";//写入文件的目录
		}
		QFile file(fileName);
		file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
		QTextStream out(&file);
		out << time << "," << num << "," << "null" /*圆心X*/ << "," << "null" /*圆心Y*/ << "," << "null"/*圆心Z*/ << "," << "false"/*圆心拟合状态*/ << ","
			<< "null"/*法矢A*/ << "," << "null"/*法矢B*/ << "," << "null"/*法矢C*/ << "," << "null" /*边缘点到圆心的平均距离*/ << "\n";
		file.close();
		emit sendProcessingResult(planeparamStatus, planeparam, centerResult, borderCloud_ptr, num);
	}
}