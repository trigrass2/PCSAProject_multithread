//#define SETUP
#define READIMAGE
#define PROGRESS
#include "displaygui.h"
using  std::cout;
using  std::endl;

DisplayGUI::DisplayGUI(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	//初始化PCL显示控件
	viewer_1.reset(new pcl::visualization::PCLVisualizer("viewer1", false));
	ui.PCLwidget_1->SetRenderWindow(viewer_1->getRenderWindow());
	viewer_1->setupInteractor(ui.PCLwidget_1->GetInteractor(), ui.PCLwidget_1->GetRenderWindow());
	ui.PCLwidget_1->update();
	//初始化核心算法
	alg = new CoreAlgorithm(0);
	
	//初始化网络通讯
	socket = new NetworkCommunication;
	//初始化视觉处理
	vp = new VisualProcessing;
	//初始化处理线程
	proThread = new ProcessThread();

	//连接网口
	connect(socket, &NetworkCommunication::stringReceived, this, &DisplayGUI::receiveSocket);
	//状态信息更新
	connect(this, &DisplayGUI::statusUpdate, this, &DisplayGUI::updateStatusInformation);
	connect(proThread, &ProcessThread::sendStatusUpdate, this, &DisplayGUI::updateStatusInformation);

	//connect(cap, &CameraCapture::sendInfo, this, &DisplayGUI::updateStatusInformation);
	connect(socket, &NetworkCommunication::sendInfo, this, &DisplayGUI::updateStatusInformation);
	//connect(motion, &MotionControl::sendInfo, this, &DisplayGUI::updateStatusInformation);

	/*connect(cap, &CameraCapture::toPLCInfo, serial, &SerialCommunication::sendPlC);
	connect(motion, &MotionControl::toPLCInfo, serial, &SerialCommunication::sendPlC);*/
	
	connect(this, &DisplayGUI::sendProcessingInformation, vp, &VisualProcessing::receiveInformation);
	connect(vp, &VisualProcessing::sendProcessingResult, this, &DisplayGUI::receiveProcessingResult);
	connect(proThread, &ProcessThread::sendProcessingResult, this, &DisplayGUI::receiveProcessingResult);

	connect(this, &DisplayGUI::sendStepNum, vp, &VisualProcessing::receiveStepNum);
	connect(vp, &VisualProcessing::sendStepParam, this, &DisplayGUI::receiveStepParam);
	
	connect(ui.networkAction, SIGNAL(triggered()), this, SLOT(networkActionSlot()));
	connect(ui.motionAction, &QAction::triggered, proThread, &ProcessThread::motionActionSlot);
	connect(ui.cameraAction, &QAction::triggered, proThread, &ProcessThread::cameraActionSlot);
	connect(ui.eyeHandAction, SIGNAL(triggered()), this, SLOT(eyeHandActionSlot()));
	connect(ui.visualProcessingAction, SIGNAL(triggered()), this, SLOT(visualProcessingActionSlot()));
	connect(ui.displayDataAction, SIGNAL(triggered()), this, SLOT(displayDataActionSlot()));

	connect(this, &DisplayGUI::sendLeftGratingData, vp, &VisualProcessing::receiveLeftGratingData);
	connect(this, &DisplayGUI::sendRightGratingData, vp, &VisualProcessing::receiveRightGratingData);
	connect(this, &DisplayGUI::sendRobotPose, vp, &VisualProcessing::receiveRobotPose);
	
	connect(proThread, &ProcessThread::sendImageAndSave, this, &DisplayGUI::receiveImageAndSave);
	//测试使用，获取图片序号
	connect(proThread, &ProcessThread::sendImageNum, this, &DisplayGUI::receiveImageNum);
	connect(this, &DisplayGUI::sendImageNum, proThread, &ProcessThread::receiveImageNum);
	//结束线程
	connect(proThread, &ProcessThread::endThread, this, &DisplayGUI::endThread);
	
	//样式表
	ui.LeftImageFirstlabel->setFrameShape(QFrame::Box);
	ui.LeftImageFirstlabel->setStyleSheet("border: 2px solid  rgb(0, 0, 0)");
	ui.LeftImageFirstlabel->setStyleSheet("background-color: rgb(0, 0, 0);");

	ui.LeftImageSecondlabel->setFrameShape(QFrame::Box);
	ui.LeftImageSecondlabel->setStyleSheet("border: 2px solid  rgb(0, 0, 0)");
	ui.LeftImageSecondlabel->setStyleSheet("background-color: rgb(0, 0, 0);");

	ui.RightImageFirstlabel->setFrameShape(QFrame::Box);
	ui.RightImageFirstlabel->setStyleSheet("border: 2px solid  rgb(0, 0, 0)");
	ui.RightImageFirstlabel->setStyleSheet("background-color: rgb(0, 0, 0);");

	ui.RightImageSecondlabel->setFrameShape(QFrame::Box);
	ui.RightImageSecondlabel->setStyleSheet("border: 2px solid  rgb(0, 0, 0)");
	ui.RightImageSecondlabel->setStyleSheet("background-color: rgb(0, 0, 0);");
	//tab页
	ui.toolBox->setCurrentIndex(0);
	ui.tabWidget->setCurrentIndex(0);
	ui.tabWidget_2->setCurrentIndex(0);
	ui.tabWidget_3->setCurrentIndex(0);
	//读取保存文件的文件夹名
	QString fileName = "ConfigFile/engineNumFileName.txt";
	QFile file(fileName);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		QMessageBox::warning(this, "Warnning", "engineNumFileName File can't open", QMessageBox::Yes);
	}
	QTextStream in(&file);
	engineNumFileName = in.readLine();
	file.close();
	/*vp->engineNumFileName = engineNumFileName;*///在这里程序可编译，但是运行时崩溃，于是将这条语句放在了receiveSocket（）位置
}

DisplayGUI::~DisplayGUI()
{
	
	delete alg;
	delete socket;
	delete vp;
	delete proThread;
	
	alg = nullptr;
	socket = nullptr;
	vp = nullptr;
	proThread = nullptr;
}

void DisplayGUI::networkActionSlot()
{
	socket->show();
}

void DisplayGUI::receiveSocket(QString info)
{
	resolveSignal(info);
	//socketNum = info.toInt();
	cout << "输入命令 = " << socketNum << endl;
	vp->engineNumFileName = engineNumFileName;
	if (engineVersion == "H")
	{
		switch (socketNum)
		{
		case 101:{//检测发动机止口法矢
					 cout << "101" << endl;
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(0);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步1第一部分,检测发动机止口法矢"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 HObject inputImageFirst, inputImageSecond;
					 ReadImage(&inputImageFirst, "edge/LeftFirst1.tif");
					 ReadImage(&inputImageSecond, "edge/LeftSecond1.tif");
					 proThread->inputImageFirst = inputImageFirst;
					 proThread->inputImageSecond = inputImageSecond;
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步1第一部分结束"), 0);
					 break;
		}
		case 102:{//检测前传动箱止口面圆心1
					 cout << "103" << endl;
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(0);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步1第二部分,检测前传动箱止口面圆心1"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步1第二部分结束"), 0);
					 break;
		}
		case 103:{//检测前传动箱止口面圆心2，圆心1，2相减
					 cout << "102" << endl;
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(0);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步1第三部分,检测前传动箱止口面圆心2，圆心1，2相减"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步1第二部分结束"), 0);
					 break;
		}
		case 201:{//检测发动机止口平面法矢和圆心坐标
					 cout << "201" << endl;
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(1);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步2第一部分,检测发动机止口平面法矢和圆心坐标"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步2第一部分结束"), 0);
					 break;
		}
		case 202:{//检测前传动箱止口平面法矢和圆心坐标
					 cout << "202" << endl;
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(1);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步2第二部分,检测前传动箱止口平面法矢和圆心坐标"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步2第二部分结束"), 0);
					 break;
		}
		case 203:{//检测发动机止口一个法兰孔圆心坐标
					 cout << "203" << endl;
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(1);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步2第三部分,检测发动机止口一个法兰孔圆心坐标"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步2第一部分结束"), 0);
					 break;
		}
		case 204:{//检测前传动箱止口一个法兰孔圆心坐标
					 cout << "204" << endl;
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(1);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步2第四部分,检测前传动箱止口一个法兰孔圆心坐标"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步2第二部分结束"), 0);
					 break;
		}
		case 301:{//检测综合传送箱止口平面法矢和圆心坐标
					 cout << "301" << endl;
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(2);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步3第一部分,检测综合传送箱止口平面法矢和圆心坐标"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步3第一部分结束"), 0);
					 break;
		}
		case 302:{//检测前传动箱止口平面法矢和圆心坐标
					 cout << "302" << endl;
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(2);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步3第二部分,检测前传动箱止口平面法矢和圆心坐标"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步3第二部分结束"), 0);
					 break;
		}
		case 303:{//检测综合传送箱止口一个法兰孔圆心坐标
					 cout << "303" << endl;
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(2);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步3第三部分,检测综合传送箱止口一个法兰孔圆心坐标"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步3第一部分结束"), 0);
					 break;
		}
		case 304:{//检测前传动箱止口一个法兰孔圆心坐标
					 cout << "304" << endl;
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(2);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步3第四部分,检测前传动箱止口一个法兰孔圆心坐标"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步3第二部分结束"), 0);
					 break;
		}
		case 401:{//检测综传动箱花键孔的平面法矢和圆心； 
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(3);
					 cout << "401" << endl;
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步4第一部分,检测综传动箱花键孔的平面法矢和圆心"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步4第一部分结束"), 0);

					 break;
		}
		case 402:{//检测前传动箱花键轴的平面法矢和圆心； 
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(3);
					 cout << "402" << endl;
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步4第二部分,检测前传动箱花键轴的平面法矢和圆心"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步4第二部分结束"), 0);

					 break;
		}
		case 403:{//检测综传动箱花键相位角
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(3);
					 cout << "403" << endl;
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步4第三部分,检测综传动箱花键相位角"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 statusUpdate(QStringLiteral("工步4第一部分结束"), 0);

					 break;
		}
		case 404:{//检测前传动箱花键相位角
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(3);
					 cout << "404" << endl;
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步4第四部分，检测前传动箱花键相位角"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步4第二部分结束"), 0);

					 break;
		}
		case 501:{//检测发动机花键孔的平面法矢和圆心
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(4);
					 cout << "501" << endl;
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步5第一部分,检测发动机花键孔的平面法矢和圆心"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步5第一部分结束"), 0);

					 break;
		}
		case 502:{//检测前传动箱花键轴的平面法矢和圆心
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(4);
					 cout << "502" << endl;
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步5第二部分,检测前传动箱花键轴的平面法矢和圆心"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步5第二部分结束"), 0);

					 break;
		}
		case 503:{//检测发动机花键相位角
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(4);
					 cout << "503" << endl;
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步5第一部分,检测发动机花键相位角"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步5第一部分结束"), 0);

					 break;
		}
		case 504:{//检测前传动箱花键相位角
					 ui.toolBox->setCurrentIndex(0);
					 ui.tabWidget_2->setCurrentIndex(4);
					 cout << "504" << endl;
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步5第二部分，检测前传动箱花键相位角"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 //statusUpdate(QStringLiteral("工步5第二部分结束"), 0);

					 break;
		}
		default:{
					cout << "输入命令错误" << endl;
					statusUpdate(QStringLiteral("输入命令错误"), 1);
					socket->send("NG");
					break;
		}

		}
	}
	else if (engineVersion == "V")
	{
		switch (socketNum)
		{
		case 101:{
					 cout << "101" << endl;
					 ui.toolBox->setCurrentIndex(1);
					 ui.tabWidget_3->setCurrentIndex(0);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步1第一部分"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);;
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 statusUpdate(QStringLiteral("工步1第一部分结束"), 0);
					 break;
		}
		case 102:{
					 cout << "102" << endl;
					 ui.toolBox->setCurrentIndex(1);
					 ui.tabWidget_3->setCurrentIndex(0);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步1第二部分"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);;
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 statusUpdate(QStringLiteral("工步1第二部分结束"), 0);
					 break;
		}
		case 201:{
					 cout << "201" << endl;
					 ui.toolBox->setCurrentIndex(1);
					 ui.tabWidget_3->setCurrentIndex(1);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步2第一部分"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 statusUpdate(QStringLiteral("工步2第一部分结束"), 0);
					 break;
		}
		case 202:{
					 cout << "202" << endl;
					 ui.toolBox->setCurrentIndex(1);
					 ui.tabWidget_3->setCurrentIndex(1);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步2第二部分"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 statusUpdate(QStringLiteral("工步2第二部分结束"), 0);
					 break;
		}
		case 301:{
					 cout << "301" << endl;
					 ui.toolBox->setCurrentIndex(1);
					 ui.tabWidget_3->setCurrentIndex(2);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步3第一部分"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 statusUpdate(QStringLiteral("工步3第一部分结束"), 0);
					 break;
		}
		case 302:{
					 cout << "302" << endl;
					 ui.toolBox->setCurrentIndex(1);
					 ui.tabWidget_3->setCurrentIndex(2);
					 socket->send("OK");
					 statusUpdate(QStringLiteral("开始工步3第二部分"), 0);
					 proThread->threadStatus = true;
					 proThread->stepNum = stepName(socketNum);
					 proThread->imageNum = ui.numEdit->text().toInt();
					 proThread->start();
					 statusUpdate(QStringLiteral("工步3第二部分结束"), 0);
					 break;
		}
		default:{
					cout << "输入命令错误" << endl;
					statusUpdate(QStringLiteral("输入命令错误"), 1);
					socket->send("NG");
					break;
		}
		}
	}
	else
	{
		if (socketNum == 0)
		{

			socket->send("OK");
			statusUpdate(QStringLiteral("创建文件夹成功"), 0);

		}
		else
		{
			cout << "输入命令错误" << endl;
			statusUpdate(QStringLiteral("输入命令错误"), 1);
			socket->send("NG");
		}

	}


}

void DisplayGUI::updateStatusInformation(QString Info,int num)
{
	QDateTime CurrentTime = QDateTime::currentDateTime();
	QString Timestr = CurrentTime.toString("yyyy-MM-dd hh:mm:ss"); // 设置显示的格式
	QString statusStr = Timestr + ": " + Info;;
	ui.statusEdit->setCurrentFont(QFont("Helvetica [Cronyx]", 12));
	if (num == 0){
		ui.statusEdit->setTextColor("black");
		ui.statusEdit->append(statusStr);
	}
	if (num == 1){
		ui.statusEdit->setTextColor("red");
		ui.statusEdit->append(statusStr);
	}
	QString fileName = engineNumFileName+"/JournalFile.txt";//写入文件的目录
	QFile file(fileName);
	file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
	QTextStream out(&file);
	out << statusStr << "\n";
	file.close();
}

void DisplayGUI::displayCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_ptr, CoreAlgorithm::StereoCircle centerResult)
{
		// 创建圆心点云（指针） 
		pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		//定义点云大小，将生成的空间圆圆心坐标赋给点云
		center_cloud->resize(1);
		//圆心坐标赋值
		{
			(*center_cloud)[0].x = centerResult.center.x;
			(*center_cloud)[0].y = centerResult.center.y;
			(*center_cloud)[0].z = centerResult.center.z;
		}
		//移除窗口点云
		viewer_1->removeAllPointClouds();
		//点云设置
		//设置点云颜色
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(Cloud_ptr, 255, 255, 255);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> center_cloud_color(Cloud_ptr, 255, 0, 0);
		//点云颜色渲染	
		viewer_1->addPointCloud(Cloud_ptr, cloud_color, "cloud");
		viewer_1->addPointCloud(center_cloud, center_cloud_color, "center_cloud");
		//设置点云大小
		viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "center_cloud");
		viewer_1->resetCamera();
		ui.PCLwidget_1->update();
		
}

void DisplayGUI::displayCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr LeftCloud_ptr/*左侧点云*/, CoreAlgorithm::StereoCircle leftCenterResult/*左侧圆心*/,
	pcl::PointCloud<pcl::PointXYZ>::Ptr RightCloud_ptr/*右侧点云*/, CoreAlgorithm::StereoCircle RightCenterResult/*右侧圆心*/)
{
	// 创建圆心点云（指针） 
	pcl::PointCloud<pcl::PointXYZ>::Ptr left_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr right_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//定义点云大小，将生成的空间圆圆心坐标赋给点云
	left_center_cloud->resize(1);
	right_center_cloud->resize(1);
	//圆心坐标赋值
	{
		(*left_center_cloud)[0].x = leftCenterResult.center.x;
		(*left_center_cloud)[0].y = leftCenterResult.center.y;
		(*left_center_cloud)[0].z = leftCenterResult.center.z;

		(*right_center_cloud)[0].x = RightCenterResult.center.x;
		(*right_center_cloud)[0].y = RightCenterResult.center.y;
		(*right_center_cloud)[0].z = RightCenterResult.center.z;

	}
	//移除窗口点云
	viewer_1->removeAllPointClouds();
	//点云设置
	//设置点云颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(LeftCloud_ptr, 255, 255, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> leftCenter_cloud_color(left_center_cloud, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> righrCenter_cloud_color(right_center_cloud, 0, 255, 0);
	//点云颜色渲染	
	viewer_1->addPointCloud(LeftCloud_ptr, cloud_color, "leftCloud");
	viewer_1->addPointCloud(RightCloud_ptr, cloud_color, "rightCloud");
	viewer_1->addPointCloud(left_center_cloud, leftCenter_cloud_color, "leftCenter_cloud");
	viewer_1->addPointCloud(right_center_cloud, righrCenter_cloud_color, "rightCenter_cloud");
	//设置点云大小
	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "leftCloud");
	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rightCloud");
	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "leftCenter_cloud");
	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "rightCenter_cloud");
	viewer_1->resetCamera();
	ui.PCLwidget_1->update();
}

void DisplayGUI::displayCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr LeftCloud_ptr/*左侧点云*/, 
	CoreAlgorithm::StereoCircle leftCenterResult/*左侧圆心*/, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr RightCloud_ptr/*右侧点云*/, 
	CoreAlgorithm::StereoCircle RightCenterResult/*右侧圆心*/, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr LeftFlangeHoleCloud_ptr/*左侧法兰孔点云*/, 
	CoreAlgorithm::StereoCircle leftFlangeHoleCenterResult/*左侧法兰孔圆心*/, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr RightFlangeHoleCloud_ptr/*右侧法兰孔点云*/, 
	CoreAlgorithm::StereoCircle RightFlangeHoleCenterResult/*右侧法兰孔圆心*/)
{
	// 创建圆心点云（指针） 
	//止口圆心
	pcl::PointCloud<pcl::PointXYZ>::Ptr left_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr right_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//法兰孔圆心
	pcl::PointCloud<pcl::PointXYZ>::Ptr leftFlangeHole_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr rightFlangeHole_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//定义点云大小，将生成的空间圆圆心坐标赋给点云
	left_center_cloud->resize(1);
	right_center_cloud->resize(1);
	leftFlangeHole_center_cloud->resize(1);
	rightFlangeHole_center_cloud->resize(1);
	//圆心坐标赋值
	{
		(*left_center_cloud)[0].x = leftCenterResult.center.x;
		(*left_center_cloud)[0].y = leftCenterResult.center.y;
		(*left_center_cloud)[0].z = leftCenterResult.center.z;

		(*right_center_cloud)[0].x = RightCenterResult.center.x;
		(*right_center_cloud)[0].y = RightCenterResult.center.y;
		(*right_center_cloud)[0].z = RightCenterResult.center.z;

		(*leftFlangeHole_center_cloud)[0].x = leftFlangeHoleCenterResult.center.x;
		(*leftFlangeHole_center_cloud)[0].y = leftFlangeHoleCenterResult.center.y;
		(*leftFlangeHole_center_cloud)[0].z = leftFlangeHoleCenterResult.center.z;

		(*rightFlangeHole_center_cloud)[0].x = RightFlangeHoleCenterResult.center.x;
		(*rightFlangeHole_center_cloud)[0].y = RightFlangeHoleCenterResult.center.y;
		(*rightFlangeHole_center_cloud)[0].z = RightFlangeHoleCenterResult.center.z;

	}
	//移除窗口点云
	viewer_1->removeAllPointClouds();
	//点云设置
	//设置点云颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(LeftCloud_ptr, 255, 255, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> leftCenter_cloud_color(left_center_cloud, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> righrCenter_cloud_color(right_center_cloud, 0, 255, 0);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> leftFlangeHolecenter_cloud_color(leftFlangeHole_center_cloud, 0, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rightFlangeHolecenter_cloud_color(leftFlangeHole_center_cloud, 255, 255, 0);
	//点云颜色渲染	
	viewer_1->addPointCloud(LeftCloud_ptr, cloud_color, "leftCloud");
	viewer_1->addPointCloud(RightCloud_ptr, cloud_color, "rightCloud");
	viewer_1->addPointCloud(left_center_cloud, leftCenter_cloud_color, "leftCenter_cloud");
	viewer_1->addPointCloud(right_center_cloud, righrCenter_cloud_color, "rightCenter_cloud");

	viewer_1->addPointCloud(LeftFlangeHoleCloud_ptr, cloud_color, "leftFlangeHoleCloud");
	viewer_1->addPointCloud(RightFlangeHoleCloud_ptr, cloud_color, "rightFlangeHoleCloud");
	viewer_1->addPointCloud(leftFlangeHole_center_cloud, leftFlangeHolecenter_cloud_color, "leftFlangeHoleCenter_cloud");
	viewer_1->addPointCloud(rightFlangeHole_center_cloud, rightFlangeHolecenter_cloud_color, "rightFlangeHoleCenter_cloud");
	//设置点云大小
	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "leftCloud");
	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rightCloud");
	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "leftCenter_cloud");
	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "rightCenter_cloud");

	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "leftFlangeHoleCloud");
	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rightFlangeHoleCloud");
	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "leftFlangeHoleCenter_cloud");
	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "rightFlangeHoleCenter_cloud");
	viewer_1->resetCamera();
	ui.PCLwidget_1->update();
}

void DisplayGUI::displayCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr LeftCloud_ptr/*左侧点云*/,
	CoreAlgorithm::StereoCircle leftCenterResult/*左侧圆心*/, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr RightCloud_ptr/*右侧点云*/, 
	CoreAlgorithm::StereoCircle RightCenterResult/*右侧圆心*/, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr FlangeHoleCloud_ptr/*某侧法兰孔点云*/, 
	CoreAlgorithm::StereoCircle FlangeHoleCenterResult/*某侧法兰孔圆心*/)
{
	// 创建圆心点云（指针） 
	//止口圆心
	pcl::PointCloud<pcl::PointXYZ>::Ptr left_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr right_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//法兰孔圆心
	pcl::PointCloud<pcl::PointXYZ>::Ptr FlangeHole_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//定义点云大小，将生成的空间圆圆心坐标赋给点云
	left_center_cloud->resize(1);
	right_center_cloud->resize(1);
	FlangeHole_center_cloud->resize(1);
	FlangeHole_center_cloud->resize(1);
	//圆心坐标赋值
	{
		(*left_center_cloud)[0].x = leftCenterResult.center.x;
		(*left_center_cloud)[0].y = leftCenterResult.center.y;
		(*left_center_cloud)[0].z = leftCenterResult.center.z;

		(*right_center_cloud)[0].x = RightCenterResult.center.x;
		(*right_center_cloud)[0].y = RightCenterResult.center.y;
		(*right_center_cloud)[0].z = RightCenterResult.center.z;

		(*FlangeHole_center_cloud)[0].x = FlangeHoleCenterResult.center.x;
		(*FlangeHole_center_cloud)[0].y = FlangeHoleCenterResult.center.y;
		(*FlangeHole_center_cloud)[0].z = FlangeHoleCenterResult.center.z;

	}
	//移除窗口点云
	viewer_1->removeAllPointClouds();
	//点云设置
	//设置点云颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(LeftCloud_ptr, 255, 255, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> leftCenter_cloud_color(left_center_cloud, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> righrCenter_cloud_color(right_center_cloud, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> FlangeHolecenter_cloud_color(FlangeHole_center_cloud, 0, 0, 255);
	//点云颜色渲染	
	viewer_1->addPointCloud(LeftCloud_ptr, cloud_color, "leftCloud");
	viewer_1->addPointCloud(RightCloud_ptr, cloud_color, "rightCloud");
	viewer_1->addPointCloud(left_center_cloud, leftCenter_cloud_color, "leftCenter_cloud");
	viewer_1->addPointCloud(right_center_cloud, righrCenter_cloud_color, "rightCenter_cloud");

	viewer_1->addPointCloud(FlangeHoleCloud_ptr, cloud_color, "FlangeHoleCloud");
	viewer_1->addPointCloud(FlangeHole_center_cloud, FlangeHolecenter_cloud_color, "FlangeHoleCenter_cloud");

	//设置点云大小
	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "leftCloud");
	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rightCloud");
	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "leftCenter_cloud");
	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "rightCenter_cloud");

	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "FlangeHoleCloud");
	viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "FlangeHoleCenter_cloud");
	viewer_1->resetCamera();
	ui.PCLwidget_1->update();
}

void DisplayGUI::displayImage(HObject Image, int num)
{
	Mat MImage;
	MImage = HImageToIplImage(Image);
	QImage img;
	img = cvMat2QImage(MImage);
	if (num == 101){
		ui.LeftImageFirstlabel->clear();
		ui.LeftImageFirstlabel->setPixmap(QPixmap::fromImage(img).scaled(QSize(MImage.cols / 8, MImage.rows / 8)));
		ui.LeftImageFirstlabel->resize(ui.LeftImageFirstlabel->pixmap()->size());
	}
	else if (num == 102){
		ui.LeftImageSecondlabel->clear();
		ui.LeftImageSecondlabel->setPixmap(QPixmap::fromImage(img).scaled(QSize(MImage.cols / 8, MImage.rows / 8)));
		ui.LeftImageSecondlabel->resize(ui.LeftImageSecondlabel->pixmap()->size());
	}
	else if (num == 201){
		ui.RightImageFirstlabel->clear();
		ui.RightImageFirstlabel->setPixmap(QPixmap::fromImage(img).scaled(QSize(MImage.cols / 8, MImage.rows / 8)));
		ui.RightImageFirstlabel->resize(ui.RightImageFirstlabel->pixmap()->size());
	}
	else if (num == 202){
		ui.RightImageSecondlabel->clear();
		ui.RightImageSecondlabel->setPixmap(QPixmap::fromImage(img).scaled(QSize(MImage.cols / 8, MImage.rows / 8)));
		ui.RightImageSecondlabel->resize(ui.RightImageSecondlabel->pixmap()->size());
	}
}

cv::Mat DisplayGUI::HImageToIplImage(HObject &Hobj)
{
	HTuple htCh = HTuple();
	HTuple cType;
	Mat Image;
	ConvertImageType(Hobj, &Hobj, "byte");
	CountChannels(Hobj, &htCh);
	HTuple wid;
	HTuple hgt;
	int W, H;
	if (htCh[0].I() == 1)
	{
		HTuple ptr;
		GetImagePointer1(Hobj, &ptr, &cType, &wid, &hgt);
		W = (Hlong)wid;
		H = (Hlong)hgt;
		Image.create(H, W, CV_8UC1);
		uchar* pdata = (uchar*)ptr[0].I();
		memcpy(Image.data, pdata, W*H);
	}
	else if (htCh[0].I() == 3)
	{
		HTuple ptrR, ptrG, ptrB;
		GetImagePointer3(Hobj, &ptrR, &ptrG, &ptrB, &cType, &wid, &hgt);
		W = (Hlong)wid;
		H = (Hlong)hgt;
		Image.create(H, W, CV_8UC3);
		vector<Mat> vecM(3);
		vecM[2].create(H, W, CV_8UC1);
		vecM[1].create(H, W, CV_8UC1);
		vecM[0].create(H, W, CV_8UC1);
		uchar* pr = (uchar*)ptrR[0].I();
		uchar* pg = (uchar*)ptrG[0].I();
		uchar* pb = (uchar*)ptrB[0].I();
		memcpy(vecM[2].data, pr, W*H);
		memcpy(vecM[1].data, pg, W*H);
		memcpy(vecM[0].data, pb, W*H);
		merge(vecM, Image);
	}
	return Image;
}

QImage DisplayGUI::cvMat2QImage(const cv::Mat& mat)
{
	// 8-bits unsigned, NO. OF CHANNELS = 1  
	if (mat.type() == CV_8UC1)
	{
		QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
		// Set the color table (used to translate colour indexes to qRgb values)  
		image.setColorCount(256);
		for (int i = 0; i < 256; i++)
		{
			image.setColor(i, qRgb(i, i, i));
		}
		// Copy input Mat  
		uchar *pSrc = mat.data;
		for (int row = 0; row < mat.rows; row++)
		{
			uchar *pDest = image.scanLine(row);
			memcpy(pDest, pSrc, mat.cols);
			pSrc += mat.step;
		}
		return image;
	}
	// 8-bits unsigned, NO. OF CHANNELS = 3  
	else if (mat.type() == CV_8UC3)
	{
		// Copy input Mat  
		const uchar *pSrc = (const uchar*)mat.data;
		// Create QImage with same dimensions as input Mat  
		QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
		return image.rgbSwapped();
	}
	else if (mat.type() == CV_8UC4)
	{

		// Copy input Mat  
		const uchar *pSrc = (const uchar*)mat.data;
		// Create QImage with same dimensions as input Mat  
		QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
		return image.copy();
	}
	else
	{
		return QImage();
	}
}


void DisplayGUI::initialization()
{
	
	
	socket->on_ConnectButton_clicked();
	
	proThread->initialization();
	socket->send("readyOK");
}


void DisplayGUI::displayDataActionSlot()
{
	QStringList strList;//参数list
	strList << "";//所需传入的命令行参数，不需参数则置空即可
	QProcess::startDetached("DisplayMessage.exe", strList);
}


void DisplayGUI::LeftScannerRun(int num, pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult)
{
//	//统计程序运行时间
//	clock_t start, finish;
//	double totaltime;
//	double startPosition;
//	double endPosition;
//	double position;
//	//***声明变量**************************************************************//
//
//	//获取工步相关参数
//	emit sendStepNum(num);
//
//	//获取导轨当前位置
//#ifndef SETUP
//	startPosition = motion->GetCurrentPositon();
//#endif //SETUP
//	
//	//声明深度图
//	HObject inputImageFirst, inputImageSecond;
//	//声明变量
//	double speed;
//	Vector<Point3f> points;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
//	start = clock();
//#ifndef SETUP
//	//光删尺计数清零
//	serial->clearGratingData();
//	Sleep(100);
//	//设置导轨运动速度
//	motion->setLeadRailVelocity(CameraNoScanSpeed);
//	//运动到第一次采集位置
//	motion->setLeadRailMotionDistance(CameraFirstNoScanDistance);
//	motion->CheckRun();
//	cout << "开始采集位置：" << endl;
//	position=motion->GetCurrentPositon();
//	Sleep(500);
//	//读取光删尺反馈导轨运动距离
//	serial->readGratingData(leftRunDis);
//	Sleep(500);
//	//获取导轨运动速度
//	speed =cap-> RateToSpeed(cap->hv_RateValue);
//	//设置导轨运动速度
//	motion->setLeadRailVelocity(speed);	
//	//获取图像
//	GrabImageStart(cap->hv_AcqHandle, -1);
//	//触发相机
//	cap->sendPLC();
//	//启动导轨开始运动, 移动距离为50mm
//	motion->setLeadRailMotionDistance(CameraScanDistance);
//	GrabImageAsync(&inputImageFirst, cap->hv_AcqHandle, -1);
//#endif //SETUP
//
//#ifndef READIMAGE
//	QString ImageFileName;
//	switch (num)
//	{
//	case 101:
//		ImageFileName = "edge/LeftFirst" + ui.numEdit->text() + ".tif";
//		break;
//	case 201:
//		ImageFileName = "edge/LeftFirst" + ui.numEdit->text() + ".tif";;
//		break;
//	case 203:
//		ImageFileName = "edge/hole1.tif";
//		break;
//	case 301:
//		ImageFileName = "edge/LeftFirst" + ui.numEdit->text() + ".tif";;
//		break;
//	case 303:
//		ImageFileName = "edge/hole1.tif";
//		break;
//	case 401:
//		ImageFileName = "gear/LeftFirst" + ui.numEdit->text() + ".tif";
//		break;
//	case 501:
//		ImageFileName = "gear/LeftFirst" + ui.numEdit->text() + ".tif";
//		break;
//	case 1010:
//		ImageFileName = "edge/LeftFirst" + ui.numEdit->text() + ".tif";
//		break;
//	case 2010:
//		ImageFileName = "edge/LeftFirst" + ui.numEdit->text() + ".tif";;
//		break;
//	case 2030:
//		ImageFileName = "edge/hole1.tif";
//		break;
//	case 3010:
//		ImageFileName = "gear/LeftFirst" + ui.numEdit->text() + ".tif";
//		break;
//		break;
//	default:
//		break;
//	}
//	
//	cout << ImageFileName.toStdString() << endl << endl;
//	QByteArray ba = ImageFileName.toLatin1();
//	const char *str = ba.data();
//	cout << ImageFileName.toStdString() << endl << endl;
//	HTuple  ImageName(str);
//	ReadImage(&inputImageFirst, ImageName);
//#endif // !READIMAGE
//	//显示图片
//	displayImage(inputImageFirst, 101);
//	//保存图片
//	SaveImage(inputImageFirst,"LeftFirst");
//	
//#ifndef SETUP
//	if (num == 203 || num == 303 || num == 2030){
//		HObject  ho_EmptyRegion;
//		GenEmptyRegion(&ho_EmptyRegion);
//		RegionToBin(ho_EmptyRegion, &inputImageSecond, 255, 0, 4096, 2000);
//	}
//	else{
//		motion->CheckRun();
//		//设置导轨运动速度
//		motion->setLeadRailVelocity(CameraNoScanSpeed);
//		//运动到第二次采集位置
//		motion->setLeadRailMotionDistance(CameraSecondNoScanDistance);
//		motion->CheckRun();
//		cout << "第二次采集位置：" << endl;
//		position = motion->GetCurrentPositon();
//		Sleep(500);
//		//读取光删尺反馈导轨运动距离
//		serial->readGratingData(leftRunDis);
//		Sleep(500);
//		//启动导轨开始运动
//		motion->setLeadRailVelocity(speed);
//		//启动导轨开始运动, 移动距离为50mm
//
//		//获取图像
//		GrabImageStart(cap->hv_AcqHandle, -1);
//		cap->sendPLC();
//		motion->setLeadRailMotionDistance(CameraScanDistance);
//		GrabImageAsync(&inputImageSecond, cap->hv_AcqHandle, -1);
//	}
//	
//
//#endif //SETUP
//
//#ifndef READIMAGE
//	switch (num)
//	{
//	case 101:
//		ImageFileName = "edge/LeftSecond" + ui.numEdit->text() + ".tif";
//		break;
//	case 201:
//		ImageFileName = "edge/LeftSecond" + ui.numEdit->text() + ".tif";
//		break;
//	case 301:
//		ImageFileName = "edge/LeftSecond" + ui.numEdit->text() + ".tif";
//		break;
//	case 401:
//		ImageFileName = "gear/LeftSecond" + ui.numEdit->text() + ".tif";
//		break;
//	case 501:
//		ImageFileName = "gear/LeftSecond" + ui.numEdit->text() + ".tif";
//		break;
//	case 1010:
//		ImageFileName = "edge/LeftSecond" + ui.numEdit->text() + ".tif";
//		break;
//	case 2010:
//		ImageFileName = "edge/LeftSecond" + ui.numEdit->text() + ".tif";
//		break;
//	case 3010:
//		ImageFileName = "gear/LeftSecond" + ui.numEdit->text() + ".tif";
//		break;
//	default:
//		break;
//	}
//	if (num != 203 && num != 303 && num != 2030){
//		ba = ImageFileName.toLatin1();
//		const char *str1 = ba.data();
//		HTuple  ImageName1(str1);
//		ReadImage(&inputImageSecond, ImageName1);
//	}
//	else{
//		HObject  ho_EmptyRegion;
//		GenEmptyRegion(&ho_EmptyRegion);
//		RegionToBin(ho_EmptyRegion, &inputImageSecond, 255, 0, 4096, 2000);
//	}
//	
//#endif //READIMAGE
//	//显示图片
//	displayImage(inputImageSecond, 102);
//	//保存图片
//	SaveImage(inputImageSecond, "LeftSecond");	
//	
//#ifndef SETUP
//	motion->CheckRun();
//	Sleep(100);
//	cout << "测试" << endl;
//	cout << "运动结束位置：" << endl;
//	position=motion->GetCurrentPositon();
//	Sleep(500);
//	//获取采集结束位置
//	endPosition = motion->GetCurrentPositon();
//	//导轨复位
//	motion->reset(startPosition,endPosition);
//	cout << "测试" << endl;
//	//发送数据
//	emit sendLeftGratingData(leftRunDis);
//	//数据清空，为下次储存做准备
//	leftRunDis.clear();
//#endif //SETUP
//
//#ifndef PROGRESS
//	switch (num)
//	{
//	case 101:
//		emit sendProcessingInformation(101, inputImageFirst, inputImageSecond);
//		break;
//	case 201:
//		emit sendProcessingInformation(201, inputImageFirst, inputImageSecond);
//		break;
//	case 203:
//		emit sendProcessingInformation(203, inputImageFirst, inputImageSecond);
//		break;
//	case 301:
//		emit sendProcessingInformation(301, inputImageFirst, inputImageSecond);
//		break;
//	case 303:
//		emit sendProcessingInformation(303, inputImageFirst, inputImageSecond);
//		break;
//	case 401:
//		emit sendProcessingInformation(401, inputImageFirst, inputImageSecond);
//		break;
//	case 501:
//		emit sendProcessingInformation(501, inputImageFirst, inputImageSecond);
//		break;
//	case 1010:
//		emit sendProcessingInformation(1010, inputImageFirst, inputImageSecond);
//		break;
//	case 2010:
//		emit sendProcessingInformation(2010, inputImageFirst, inputImageSecond);
//		break;
//	case 2030:
//		emit sendProcessingInformation(2030, inputImageFirst, inputImageSecond);
//		break;
//	case 3010:
//		emit sendProcessingInformation(3010, inputImageFirst, inputImageSecond);
//		break;
//	case 3030:
//		emit sendProcessingInformation(3030, inputImageFirst, inputImageSecond);
//		break;
//	default:
//		break;
//	}
//#endif //PROGRESS
//	cout << "over!" << endl;
//	finish = clock();
//	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
//	//清空变量
//	points.clear();
//
//	cout << "此程序的运行时间为" << totaltime << "秒！" << endl;
}

void DisplayGUI::RightScannerRun(int num, pcl::PointCloud<pcl::PointXYZ>::Ptr &borderCloud_ptr, CoreAlgorithm::StereoCircle &centerResult)
{
//	//统计程序运行时间
//	clock_t start, finish;
//	double totaltime;
//	double startPosition;
//	double endPosition;
//	double position;
//	//***声明变量**************************************************************//
//	//获取导轨当前位置
//#ifndef SETUP
//	startPosition = motion->GetCurrentPositon();
//#endif //SETUP
//	//获取工步相关参数
//	emit sendStepNum(num);
//	//声明深度图
//	HObject inputImageFirst, inputImageSecond;
//	//声明变量
//	double speed;
//	Vector<Point3f> points;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
//	start = clock();
//#ifndef SETUP
//	serial->clearGratingData();
//	Sleep(100);
//	motion->setLeadRailVelocity(CameraNoScanSpeed);
//	//运动到第一次采集位置
//	motion->setLeadRailMotionDistance(CameraFirstNoScanDistance);
//	motion->CheckRun();	
//	cout << "开始采集位置：" << endl;
//	position = motion->GetCurrentPositon();
//	Sleep(500);
//	//读取光删尺反馈导轨运动距离
//	serial->readGratingData(rightRunDis);
//	Sleep(500);
//	//获取导轨运动速度
//	speed = cap->RateToSpeed(cap->hv_RateValue);
//	//设置导轨运动速度
//	motion->setLeadRailVelocity(speed);
//	//获取图像
//	GrabImageStart(cap->hv_AcqHandle, -1);
//	cap->sendPLC();
//	//启动导轨开始运动, 移动距离为50mmCameraScanDistance
//	motion->setLeadRailMotionDistance(CameraScanDistance);
//	GrabImageAsync(&inputImageFirst, cap->hv_AcqHandle, -1);
//#endif //SETUP
//
//#ifndef READIMAGE
//	QString ImageFileName;
//	switch (num)
//	{
//	case 102:
//		ImageFileName = "edge/LeftFirst" + ui.numEdit->text() + ".tif";
//		break;
//	case 104:
//		ImageFileName = "edge/LeftFirst2.tif";
//		break;
//	case 202:
//		ImageFileName = "edge/LeftFirst" + ui.numEdit->text() + ".tif";
//		break;
//	case 204:
//		ImageFileName = "edge/hole1.tif";
//		break;
//	case 302:
//		ImageFileName = "edge/LeftFirst" + ui.numEdit->text() + ".tif";
//		break;
//	case 304:
//		ImageFileName = "edge/hole1.tif";
//		break;
//	case 402:
//		ImageFileName = "gear/RightFirst" + ui.numEdit->text() + ".tif";
//		break;
//	case 502:
//		ImageFileName = "gear/RightFirst" + ui.numEdit->text() + ".tif";
//		break;
//	case 1020:
//		ImageFileName = "edge/LeftFirst" + ui.numEdit->text() + ".tif";
//		break;
//	case 1040:
//		ImageFileName = "edge/LeftFirst2.tif";
//		break;
//	case 2020:
//		ImageFileName = "edge/LeftFirst" + ui.numEdit->text() + ".tif";
//		break;
//	case 2040:
//		ImageFileName = "edge/hole1.tif";
//		break;
//	case 3020:
//		ImageFileName = "gear/RightFirst" + ui.numEdit->text() + ".tif";
//		break;
//	default:
//		break;
//	}
//	cout << ImageFileName.toStdString() << endl << endl;
//	QByteArray ba = ImageFileName.toLatin1();
//	const char *str = ba.data();
//	cout << ImageFileName.toStdString() << endl << endl;
//	HTuple  ImageName(str);
//	ReadImage(&inputImageFirst, ImageName);
//#endif // !READIMAGE
//	//显示图片
//	displayImage(inputImageFirst, 201);
//	//保存图片
//	SaveImage(inputImageFirst, "RightFirst");
//
//#ifndef SETUP
//	if (num == 204 || num == 304||num == 2040 ){
//		HObject  ho_EmptyRegion;
//		GenEmptyRegion(&ho_EmptyRegion);
//		RegionToBin(ho_EmptyRegion, &inputImageSecond, 255, 0, 4096, 2000);
//	}
//	else{
//		motion->CheckRun();
//		Sleep(100);
//		//设置导轨运动速度
//		motion->setLeadRailVelocity(CameraNoScanSpeed);
//		//运动到第二次采集位置
//		motion->setLeadRailMotionDistance(CameraSecondNoScanDistance);
//		motion->CheckRun();
//		cout << "第二次采集位置：" << endl;
//		position = motion->GetCurrentPositon();
//		Sleep(500);
//		//读取光删尺反馈导轨运动距离
//		serial->readGratingData(rightRunDis);
//		Sleep(500);
//		//启动导轨开始运动
//		motion->setLeadRailVelocity(speed);
//		//启动导轨开始运动, 移动距离为50mm
//
//		//获取图像
//		GrabImageStart(cap->hv_AcqHandle, -1);
//		cap->sendPLC();
//		motion->setLeadRailMotionDistance(CameraScanDistance);
//		GrabImageAsync(&inputImageSecond, cap->hv_AcqHandle, -1);
//	}
//
//
//#endif //SETUP
//
//#ifndef READIMAGE
//	switch (num)
//	{
//	case 102:
//		ImageFileName = "edge/LeftSecond" + ui.numEdit->text() + ".tif";
//		break;
//	case 104:
//		ImageFileName = "edge/LeftSecond2.tif";
//		break;
//	case 202:
//		ImageFileName = "edge/LeftSecond" + ui.numEdit->text() + ".tif";
//		break;
//	/*case 204:
//		ImageFileName = "edge/LeftSecond" + ui.numEdit->text() + ".tif";
//		break;*/
//	case 302:
//		ImageFileName = "edge/LeftSecond" + ui.numEdit->text() + ".tif";
//		break;
//	case 402:
//		ImageFileName = "gear/RightSecond" + ui.numEdit->text() + ".tif";
//		break;
//	case 502:
//		ImageFileName = "gear/RightSecond" + ui.numEdit->text() + ".tif";
//		break;
//	case 1020:
//		ImageFileName = "edge/LeftSecond" + ui.numEdit->text() + ".tif";
//		break;
//	case 1040:
//		ImageFileName = "edge/LeftSecond2.tif";
//		break;
//	case 2020:
//		ImageFileName = "edge/LeftSecond" + ui.numEdit->text() + ".tif";
//		break;
//	case 3020:
//		ImageFileName = "gear/RightSecond" + ui.numEdit->text() + ".tif";
//		break;
//	default:
//		break;
//	}
//	if (num != 204 && num != 304 && num != 2040){
//		ba = ImageFileName.toLatin1();
//		const char *str1 = ba.data();
//		HTuple  ImageName1(str1);
//		ReadImage(&inputImageSecond, ImageName1);
//	}
//	else{
//		HObject  ho_EmptyRegion;
//		GenEmptyRegion(&ho_EmptyRegion);
//		RegionToBin(ho_EmptyRegion, &inputImageSecond, 255, 0, 4096, 2000);
//	}
//
//	
//#endif //READIMAGE
//
//
//	//显示图片
//	displayImage(inputImageSecond, 202);
//	//保存图片
//	SaveImage(inputImageSecond, "RightSecond");
//#ifndef SETUP
//	motion->CheckRun();
//	Sleep(100);
//	cout << "测试" << endl;
//	cout << "运动结束位置：" << endl;
//	position = motion->GetCurrentPositon();
//	Sleep(500);
//	//获取采集结束位置
//	endPosition = motion->GetCurrentPositon();
//	//导轨复位
//	motion->reset(startPosition, endPosition);
//	cout << "测试" << endl;
//	//发送数据
//	emit sendRightGratingData(rightRunDis);
//	//数据清空，为下次储存做准备
//	rightRunDis.clear();
//#endif //SETUP
//
//#ifndef PROGRESS 
//	switch (num)
//	{
//	case 102:
//		emit sendProcessingInformation(102, inputImageFirst, inputImageSecond);
//		break;
//	case 104:
//		emit sendProcessingInformation(104, inputImageFirst, inputImageSecond);
//		break;
//	case 202:
//		emit sendProcessingInformation(202, inputImageFirst, inputImageSecond);
//		break;
//	case 204:
//		emit sendProcessingInformation(204, inputImageFirst, inputImageSecond);
//		break;
//	case 302:
//		emit sendProcessingInformation(302, inputImageFirst, inputImageSecond);
//		break;
//	case 304:
//		emit sendProcessingInformation(304, inputImageFirst, inputImageSecond);
//		break;
//	case 402:
//		emit sendProcessingInformation(402, inputImageFirst, inputImageSecond);
//		break;
//	case 502:
//		emit sendProcessingInformation(502, inputImageFirst, inputImageSecond);
//		break;
//	case 1020:
//		emit sendProcessingInformation(1020, inputImageFirst, inputImageSecond);
//		break;
//	case 1040:
//		emit sendProcessingInformation(1040, inputImageFirst, inputImageSecond);
//		break;
//	case 2020:
//		emit sendProcessingInformation(2020, inputImageFirst, inputImageSecond);
//		break;
//	case 2040:
//		emit sendProcessingInformation(2040, inputImageFirst, inputImageSecond);
//		break;
//	case 3020:
//		emit sendProcessingInformation(3020, inputImageFirst, inputImageSecond);
//		break;
//	case 3040:
//		emit sendProcessingInformation(3040, inputImageFirst, inputImageSecond);
//		break;
//	default:
//		break;
//	}
//	
//#endif //PROGRESS
//	cout << "over!" << endl;
//	finish = clock();
//	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
//	//清空变量
//	points.clear();
//
//	cout << "此程序的运行时间为" << totaltime << "秒！" << endl;
}

/*void DisplayGUI::InitializationLineEdit()
{
	//工位1
	lineEdit[0] = ui.LeftRunFirst_StepOne;
	lineEdit[1] = ui.LeftRunSecond_StepOne;

	lineEdit[2] = ui.RightRunFirst_StepOne;
	lineEdit[3] = ui.RightRunSecond_StepOne;

	lineEdit[4] = ui.LeftRestrictRadius_StepOne;
	lineEdit[5] = ui.RightRestrictRadius_StepOne;

	lineEdit[6] = ui.LeftCameraScan_StepOne;
	lineEdit[7] = ui.RightCameraScan_StepOne;

	//工位2
	lineEdit[8] = ui.LeftRunFirst_StepTwo;
	lineEdit[9] = ui.LeftRunSecond_StepTwo;

	lineEdit[10] = ui.RightRunFirst_StepTwo;
	lineEdit[11] = ui.RightRunSecond_StepTwo;

	lineEdit[12] = ui.LeftRestrictRadius_StepTwo;
	lineEdit[13] = ui.RightRestrictRadius_StepTwo;

	lineEdit[14] = ui.LeftCameraScan_StepTwo;
	lineEdit[15] = ui.RightCameraScan_StepTwo;

	//工位3
	lineEdit[16] = ui.LeftRunFirst_StepThree;
	lineEdit[17] = ui.LeftRunSecond_StepThree;

	lineEdit[18] = ui.RightRunFirst_StepThree;
	lineEdit[19] = ui.RightRunSecond_StepThree;

	lineEdit[20] = ui.LeftRestrictRadiu_StepThree;
	lineEdit[21] = ui.RightRestrictRadius_StepThree;

	lineEdit[22] = ui.LeftCameraScan_StepThree;
	lineEdit[23] = ui.RightCameraScan_StepThree;

	//工位4
	lineEdit[24] = ui.LeftRunFirst_StepFour;
	lineEdit[25] = ui.LeftRunSecond_StepFour;

	lineEdit[26] = ui.RightRunFirst_StepFour;
	lineEdit[27] = ui.RightRunSecond_StepFour;

	lineEdit[28] = ui.LeftRestrictRadius_StepFour;
	lineEdit[29] = ui.RightRestrictRadius_StepFour;

	lineEdit[30] = ui.LeftCameraScan_StepFour;
	lineEdit[31] = ui.RightCameraScan_StepFour;

	//工位5
	lineEdit[32] = ui.LeftRunFirst_StepFive;
	lineEdit[33] = ui.LeftRunSecond_StepFive;

	lineEdit[34] = ui.RightRunFirst_StepFive;
	lineEdit[35] = ui.RightRunSecond_StepFive;

	lineEdit[36] = ui.LeftRestrictRadius_StepFive;
	lineEdit[37] = ui.RightRestrictRadius_StepFive;

	lineEdit[38] = ui.LeftCameraScan_StepFive;
	lineEdit[39] = ui.RightCameraScan_StepFive;

	//右测头坐标系到左侧头坐标系变换矩阵
	lineEdit[40] = ui.RtoL_M00;
	lineEdit[41] = ui.RtoL_M01;
	lineEdit[42] = ui.RtoL_M02;
	lineEdit[43] = ui.RtoL_M03;

	lineEdit[44] = ui.RtoL_M10;
	lineEdit[45] = ui.RtoL_M11;
	lineEdit[46] = ui.RtoL_M12;
	lineEdit[47] = ui.RtoL_M13;

	lineEdit[48] = ui.RtoL_M20;
	lineEdit[49] = ui.RtoL_M21;
	lineEdit[50] = ui.RtoL_M22;
	lineEdit[51] = ui.RtoL_M23;

	lineEdit[52] = ui.RtoL_M30;
	lineEdit[53] = ui.RtoL_M31;
	lineEdit[54] = ui.RtoL_M32;
	lineEdit[55] = ui.RtoL_M33;

	//手眼标定矩阵
	lineEdit[56] = ui.EyeInHand_M00;
	lineEdit[57] = ui.EyeInHand_M01;
	lineEdit[58] = ui.EyeInHand_M02;
	lineEdit[59] = ui.EyeInHand_M03;

	lineEdit[60] = ui.EyeInHand_M10;
	lineEdit[61] = ui.EyeInHand_M11;
	lineEdit[62] = ui.EyeInHand_M12;
	lineEdit[63] = ui.EyeInHand_M13;

	lineEdit[64] = ui.EyeInHand_M20;
	lineEdit[65] = ui.EyeInHand_M21;
	lineEdit[66] = ui.EyeInHand_M22;
	lineEdit[67] = ui.EyeInHand_M23;

	lineEdit[68] = ui.EyeInHand_M30;
	lineEdit[69] = ui.EyeInHand_M31;
	lineEdit[70] = ui.EyeInHand_M32;
	lineEdit[71] = ui.EyeInHand_M33;

}*/

/*void DisplayGUI::ReadConfigParameterFile()
{
	QString fileName = "ConfigFile/ConfigInSteps.txt";
	QFile file(fileName);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		QMessageBox::warning(this, "Warnning", "File can't open", QMessageBox::Yes);
	}
	QTextStream in(&file);
	double variable;
	for (int i = 0; i < 72; i++){
		variable = in.readLine().toDouble();
		lineEdit[i]->setText(QString::number(variable));
	}
}*/

QString DisplayGUI::ReturnTime()
{
	QDateTime CurrentTime = QDateTime::currentDateTime();
	QString Timestr = CurrentTime.toString("yyyy-MM-dd hh-mm-ss"); // 设置显示的格式
	return Timestr;
}

void DisplayGUI::SaveImage(HObject Image, QString name)
{
	QString TimeStr;
	QString ImageFileName;
	Mat Img;
	TimeStr = ReturnTime();
	QDir dir;
	bool isExists = dir.exists(engineNumFileName + "/JournalImage");
	if (!isExists){
		dir.mkdir(engineNumFileName + "/JournalImage");
	}
	ImageFileName = engineNumFileName + "/JournalImage/" + TimeStr + " " + name + ".tif";
	QByteArray ba = ImageFileName.toLatin1();
	const char *str = ba.data();
	HTuple  ImageName(str);
	WriteImage(Image, "tiff", 0, ImageName);

}

void DisplayGUI::eyeHandActionSlot()
{
	QStringList strList;//参数list
	strList << "" ;//所需传入的命令行参数，不需参数则置空即可
	QProcess::startDetached("EyeinHand.exe", strList);
}

void DisplayGUI::visualProcessingActionSlot()
{
	vp->show();
}


void DisplayGUI::receiveProcessingResult(bool planeparamStatus/*平面法矢处理状态*/,
	CoreAlgorithm::PlaneNormal planeparam/*平面法矢*/, 
	CoreAlgorithm::StereoCircle centerResult/*空间圆心*/, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud_ptr/*边缘点云 PCL格式*/, 
	int num/*工步号*/)
{
	/*if (num % 2 == 1){
		leftProcessingStatus = status;
		}
		else{
		rightProcessingStatus = status;
		}*/
	/*清空变量*/
	resultMessage.clear();

	if (num == 101){
		if (planeparamStatus == true){
			//输出结果
			ui.StepOne_horizontalEngine_normal_x_Edit->setStyleSheet("color:black");
			ui.StepOne_horizontalEngine_normal_y_Edit->setStyleSheet("color:black");
			ui.StepOne_horizontalEngine_normal_z_Edit->setStyleSheet("color:black");
			ui.StepOne_horizontalEngine_normal_x_Edit->setText(QString::number(planeparam.A));
			ui.StepOne_horizontalEngine_normal_y_Edit->setText(QString::number(planeparam.B));
			ui.StepOne_horizontalEngine_normal_z_Edit->setText(QString::number(planeparam.C));
			leftPlaneNormal = planeparam;
			leftProcessingStatus = true;
			//添加到发送信息
			resultMessage.leftNormal.A = QString::number(planeparam.A);
			resultMessage.leftNormal.B = QString::number(planeparam.B);
			resultMessage.leftNormal.C = QString::number(planeparam.C);
		}
		else{
			statusUpdate(QStringLiteral("左侧采集出现问题，请调整位置重新采集！"), 1);
			ui.StepOne_horizontalEngine_normal_x_Edit->setStyleSheet("color:red");
			ui.StepOne_horizontalEngine_normal_y_Edit->setStyleSheet("color:red");
			ui.StepOne_horizontalEngine_normal_z_Edit->setStyleSheet("color:red");
			ui.StepOne_horizontalEngine_normal_x_Edit->setText("NULL");
			ui.StepOne_horizontalEngine_normal_y_Edit->setText("NULL");
			ui.StepOne_horizontalEngine_normal_z_Edit->setText("NULL");
			leftProcessingStatus = false;		
			//添加到发送信息
			resultMessage.leftNormal.A = "NULL";
			resultMessage.leftNormal.B = "NULL";
			resultMessage.leftNormal.C = "NULL";
		}
		QString info;
		info = resultMessage.leftNormal.A +","+ resultMessage.leftNormal.B +","+ resultMessage.leftNormal.C;
		socket->send(info);
	}	
	if (num == 102){
		transmissionCaseCenterResultFirst = centerResult;
		transmissionCaseCircleFirst_cloudPoints = borderCloud_ptr;
		if (transmissionCaseCenterResultFirst.status == true){
			//输出结果
			ui.StepOne_horizontalFrontDriveBox_First_center_x_Edit->setStyleSheet("color:black");
			ui.StepOne_horizontalFrontDriveBox_First_center_y_Edit->setStyleSheet("color:black");
			ui.StepOne_horizontalFrontDriveBox_First_center_z_Edit->setStyleSheet("color:black");
			ui.StepOne_horizontalFrontDriveBox_First_center_x_Edit->setText(QString::number(centerResult.center.x));
			ui.StepOne_horizontalFrontDriveBox_First_center_y_Edit->setText(QString::number(centerResult.center.y));
			ui.StepOne_horizontalFrontDriveBox_First_center_z_Edit->setText(QString::number(centerResult.center.z));
			rightProcessingStatus = true;
			//添加到发送信息
			resultMessage.rightCenter.X = QString::number(centerResult.center.x);
			resultMessage.rightCenter.Y = QString::number(centerResult.center.y);
			resultMessage.rightCenter.Z = QString::number(centerResult.center.z);
			displayCloud(transmissionCaseCircleFirst_cloudPoints, transmissionCaseCenterResultFirst);

		}
		else{
			statusUpdate(QStringLiteral("右侧圆心拟合失败，重新采集！"), 1);
			ui.StepOne_horizontalFrontDriveBox_First_center_x_Edit->setStyleSheet("color:red");
			ui.StepOne_horizontalFrontDriveBox_First_center_y_Edit->setStyleSheet("color:red");
			ui.StepOne_horizontalFrontDriveBox_First_center_z_Edit->setStyleSheet("color:red");
			ui.StepOne_horizontalFrontDriveBox_First_center_x_Edit->setText("NULL");
			ui.StepOne_horizontalFrontDriveBox_First_center_y_Edit->setText("NULL");
			ui.StepOne_horizontalFrontDriveBox_First_center_z_Edit->setText("NULL");
			rightProcessingStatus = false;
			//添加到发送信息
			resultMessage.rightCenter.X = "NULL";
			resultMessage.rightCenter.Y = "NULL";
			resultMessage.rightCenter.Z = "NULL";
		}
		//stepOneFirst();
	}
	if (num == 104){
		transmissionCaseCenterResultSecond = centerResult;
		transmissionCaseCircleSecond_cloudPoints = borderCloud_ptr;
		if (transmissionCaseCenterResultSecond.status == true)
		{
			//输出结果
			//计算方向矢量
			directionVector.x = transmissionCaseCenterResultFirst.center.x - transmissionCaseCenterResultSecond.center.x;
			directionVector.y = transmissionCaseCenterResultFirst.center.y - transmissionCaseCenterResultSecond.center.y;
			directionVector.z = transmissionCaseCenterResultFirst.center.z - transmissionCaseCenterResultSecond.center.z;
			QString fileName = engineNumFileName + "/outputFile/StepOne/frontDriveBoxDirectionVector.csv";//写入文件的目录
			QString time = ReturnTime();
			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time <<","<< directionVector.x << "," << directionVector.y << "," << directionVector.y << "\n";
			file.close();
			ui.StepOne_horizontalFrontDriveBox_Second_center_x_Edit->setStyleSheet("color:black");
			ui.StepOne_horizontalFrontDriveBox_Second_center_y_Edit->setStyleSheet("color:black");
			ui.StepOne_horizontalFrontDriveBox_Second_center_z_Edit->setStyleSheet("color:black");
			ui.StepOne_horizontalFrontDriveBox_directionVector_x_Edit->setStyleSheet("color:black");
			ui.StepOne_horizontalFrontDriveBox_directionVector_y_Edit->setStyleSheet("color:black");
			ui.StepOne_horizontalFrontDriveBox_directionVector_z_Edit->setStyleSheet("color:black");

			ui.StepOne_horizontalFrontDriveBox_Second_center_x_Edit->setText(QString::number(centerResult.center.x));
			ui.StepOne_horizontalFrontDriveBox_Second_center_y_Edit->setText(QString::number(centerResult.center.y));
			ui.StepOne_horizontalFrontDriveBox_Second_center_z_Edit->setText(QString::number(centerResult.center.z));

			ui.StepOne_horizontalFrontDriveBox_directionVector_x_Edit->setText(QString::number(directionVector.x));
			ui.StepOne_horizontalFrontDriveBox_directionVector_y_Edit->setText(QString::number(directionVector.y));
			ui.StepOne_horizontalFrontDriveBox_directionVector_z_Edit->setText(QString::number(directionVector.z));
			rightProcessingStatus = true;
			//添加到发送信息
			resultMessage.rightCenter.X = QString::number(centerResult.center.x);
			resultMessage.rightCenter.Y = QString::number(centerResult.center.y);
			resultMessage.rightCenter.Z = QString::number(centerResult.center.z);
			resultMessage.directionVector.A = QString::number(directionVector.x);
			resultMessage.directionVector.B = QString::number(directionVector.y);
			resultMessage.directionVector.C = QString::number(directionVector.z);
		}
		else{
			statusUpdate(QStringLiteral("右侧圆心拟合失败，重新采集！"), 1);
			ui.StepOne_horizontalFrontDriveBox_Second_center_x_Edit->setStyleSheet("color:red");
			ui.StepOne_horizontalFrontDriveBox_Second_center_y_Edit->setStyleSheet("color:red");
			ui.StepOne_horizontalFrontDriveBox_Second_center_z_Edit->setStyleSheet("color:red");
			ui.StepOne_horizontalFrontDriveBox_directionVector_x_Edit->setStyleSheet("color:red");
			ui.StepOne_horizontalFrontDriveBox_directionVector_y_Edit->setStyleSheet("color:red");
			ui.StepOne_horizontalFrontDriveBox_directionVector_z_Edit->setStyleSheet("color:red");

			ui.StepOne_horizontalFrontDriveBox_Second_center_x_Edit->setText("NULL");
			ui.StepOne_horizontalFrontDriveBox_Second_center_y_Edit->setText("NULL");
			ui.StepOne_horizontalFrontDriveBox_Second_center_z_Edit->setText("NULL");

			ui.StepOne_horizontalFrontDriveBox_directionVector_x_Edit->setText("NULL");
			ui.StepOne_horizontalFrontDriveBox_directionVector_y_Edit->setText("NULL");
			ui.StepOne_horizontalFrontDriveBox_directionVector_y_Edit->setText("NULL");
			rightProcessingStatus = false;
			//添加到发送信息
			resultMessage.rightCenter.X = "NULL";
			resultMessage.rightCenter.Y = "NULL";
			resultMessage.rightCenter.Z = "NULL";
			resultMessage.directionVector.A = "NULL";
			resultMessage.directionVector.B = "NULL";
			resultMessage.directionVector.C = "NULL";
		}

	}

	if (num == 201){
		leftCenterResult = centerResult;
		leftCircle_cloudPoints = borderCloud_ptr;
		leftPlaneNormal = planeparam;
		if (planeparamStatus == true)
		{
			if (leftCenterResult.status == true){
				//输出结果
				ui.StepTwo_horizontalEngine_normal_x_Edit->setStyleSheet("color:black");
				ui.StepTwo_horizontalEngine_normal_y_Edit->setStyleSheet("color:black");
				ui.StepTwo_horizontalEngine_normal_z_Edit->setStyleSheet("color:black");

				ui.StepTwo_horizontalEngine_center_x_Edit->setStyleSheet("color:black");
				ui.StepTwo_horizontalEngine_center_y_Edit->setStyleSheet("color:black");
				ui.StepTwo_horizontalEngine_center_z_Edit->setStyleSheet("color:black");
				
				ui.StepTwo_horizontalEngine_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepTwo_horizontalEngine_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepTwo_horizontalEngine_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepTwo_horizontalEngine_center_x_Edit->setText(QString::number(centerResult.center.x));
				ui.StepTwo_horizontalEngine_center_y_Edit->setText(QString::number(centerResult.center.y));
				ui.StepTwo_horizontalEngine_center_z_Edit->setText(QString::number(centerResult.center.z));
				leftProcessingStatus = true;
				//添加到发送信息
				resultMessage.leftNormal.A = QString::number(planeparam.A);
				resultMessage.leftNormal.B = QString::number(planeparam.B);
				resultMessage.leftNormal.C = QString::number(planeparam.C);
				resultMessage.leftCenter.X = QString::number(centerResult.center.x);
				resultMessage.leftCenter.Y = QString::number(centerResult.center.y);
				resultMessage.leftCenter.Z = QString::number(centerResult.center.z);

			}
			else{
				statusUpdate(QStringLiteral("左侧圆心拟合失败，重新采集！"), 1);
				ui.StepTwo_horizontalEngine_normal_x_Edit->setStyleSheet("color:black");
				ui.StepTwo_horizontalEngine_normal_y_Edit->setStyleSheet("color:black");
				ui.StepTwo_horizontalEngine_normal_z_Edit->setStyleSheet("color:black");

				ui.StepTwo_horizontalEngine_center_x_Edit->setStyleSheet("color:red");
				ui.StepTwo_horizontalEngine_center_y_Edit->setStyleSheet("color:red");
				ui.StepTwo_horizontalEngine_center_z_Edit->setStyleSheet("color:red");

				ui.StepTwo_horizontalEngine_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepTwo_horizontalEngine_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepTwo_horizontalEngine_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepTwo_horizontalEngine_center_x_Edit->setText("NULL");
				ui.StepTwo_horizontalEngine_center_y_Edit->setText("NULL");
				ui.StepTwo_horizontalEngine_center_z_Edit->setText("NULL");

				leftProcessingStatus = true;
				//添加到发送信息
				resultMessage.leftNormal.A = QString::number(planeparam.A);
				resultMessage.leftNormal.B = QString::number(planeparam.B);
				resultMessage.leftNormal.C = QString::number(planeparam.C);
				resultMessage.leftCenter.X = "NULL";
				resultMessage.leftCenter.Y = "NULL";
				resultMessage.leftCenter.Z = "NULL";
			}
		}
		else
		{
			statusUpdate(QStringLiteral("左侧采集出现问题，请调整位置重新采集！"), 1);
			ui.StepTwo_horizontalEngine_normal_x_Edit->setStyleSheet("color:red");
			ui.StepTwo_horizontalEngine_normal_y_Edit->setStyleSheet("color:red");
			ui.StepTwo_horizontalEngine_normal_z_Edit->setStyleSheet("color:red");

			ui.StepTwo_horizontalEngine_center_x_Edit->setStyleSheet("color:red");
			ui.StepTwo_horizontalEngine_center_y_Edit->setStyleSheet("color:red");
			ui.StepTwo_horizontalEngine_center_z_Edit->setStyleSheet("color:red");

			ui.StepTwo_horizontalEngine_normal_x_Edit->setText("NULL");
			ui.StepTwo_horizontalEngine_normal_y_Edit->setText("NULL");
			ui.StepTwo_horizontalEngine_normal_z_Edit->setText("NULL");

			ui.StepTwo_horizontalEngine_center_x_Edit->setText("NULL");
			ui.StepTwo_horizontalEngine_center_y_Edit->setText("NULL");
			ui.StepTwo_horizontalEngine_center_z_Edit->setText("NULL");

			leftProcessingStatus = false;
			//添加到发送信息
			resultMessage.leftNormal.A = "NULL";
			resultMessage.leftNormal.B = "NULL";
			resultMessage.leftNormal.C = "NULL";
			resultMessage.leftCenter.X = "NULL";
			resultMessage.leftCenter.Y = "NULL";
			resultMessage.leftCenter.Z = "NULL";
		}
		

	}
	if (num ==202){
		rightCenterResult = centerResult;
		rightCircle_cloudPoints = borderCloud_ptr;
		rightPlaneNormal = planeparam;
		if (planeparamStatus == true)
		{
			if (rightCenterResult.status == true){
				//输出结果
				ui.StepTwo_horizontalFrontDriveBox_normal_x_Edit->setStyleSheet("color:black");
				ui.StepTwo_horizontalFrontDriveBox_normal_y_Edit->setStyleSheet("color:black");
				ui.StepTwo_horizontalFrontDriveBox_normal_z_Edit->setStyleSheet("color:black");
				ui.StepTwo_horizontalFrontDriveBox_center_x_Edit->setStyleSheet("color:black");
				ui.StepTwo_horizontalFrontDriveBox_center_y_Edit->setStyleSheet("color:black");
				ui.StepTwo_horizontalFrontDriveBox_center_z_Edit->setStyleSheet("color:black");


				ui.StepTwo_horizontalFrontDriveBox_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepTwo_horizontalFrontDriveBox_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepTwo_horizontalFrontDriveBox_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepTwo_horizontalFrontDriveBox_center_x_Edit->setText(QString::number(centerResult.center.x));
				ui.StepTwo_horizontalFrontDriveBox_center_y_Edit->setText(QString::number(centerResult.center.y));
				ui.StepTwo_horizontalFrontDriveBox_center_z_Edit->setText(QString::number(centerResult.center.z));
				rightProcessingStatus = true;
				//添加到发送信息
				resultMessage.rightNormal.A = QString::number(planeparam.A);
				resultMessage.rightNormal.B = QString::number(planeparam.B);
				resultMessage.rightNormal.C = QString::number(planeparam.C);
				resultMessage.rightCenter.X = QString::number(centerResult.center.x);
				resultMessage.rightCenter.Y = QString::number(centerResult.center.y);
				resultMessage.rightCenter.Z = QString::number(centerResult.center.z);

			}
			else{
				statusUpdate(QStringLiteral("右侧圆心拟合失败，重新采集！"), 1);
				ui.StepTwo_horizontalFrontDriveBox_normal_x_Edit->setStyleSheet("color:black");
				ui.StepTwo_horizontalFrontDriveBox_normal_y_Edit->setStyleSheet("color:black");
				ui.StepTwo_horizontalFrontDriveBox_normal_z_Edit->setStyleSheet("color:black");
				ui.StepTwo_horizontalFrontDriveBox_center_x_Edit->setStyleSheet("color:red");
				ui.StepTwo_horizontalFrontDriveBox_center_y_Edit->setStyleSheet("color:red");
				ui.StepTwo_horizontalFrontDriveBox_center_z_Edit->setStyleSheet("color:red");

				ui.StepTwo_horizontalFrontDriveBox_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepTwo_horizontalFrontDriveBox_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepTwo_horizontalFrontDriveBox_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepTwo_horizontalFrontDriveBox_center_x_Edit->setText("NULL");
				ui.StepTwo_horizontalFrontDriveBox_center_y_Edit->setText("NULL");
				ui.StepTwo_horizontalFrontDriveBox_center_z_Edit->setText("NULL");
				rightProcessingStatus = true;
				//添加到发送信息
				resultMessage.rightNormal.A = QString::number(planeparam.A);
				resultMessage.rightNormal.B = QString::number(planeparam.B);
				resultMessage.rightNormal.C = QString::number(planeparam.C);
				resultMessage.rightCenter.X = "NULL";
				resultMessage.rightCenter.Y = "NULL";
				resultMessage.rightCenter.Z = "NULL";

			}
		}
		else
		{
			statusUpdate(QStringLiteral("右侧采集出现问题，请调整位置重新采集！"), 1);
			ui.StepTwo_horizontalFrontDriveBox_normal_x_Edit->setStyleSheet("color:red");
			ui.StepTwo_horizontalFrontDriveBox_normal_y_Edit->setStyleSheet("color:red");
			ui.StepTwo_horizontalFrontDriveBox_normal_z_Edit->setStyleSheet("color:red");
			ui.StepTwo_horizontalFrontDriveBox_center_x_Edit->setStyleSheet("color:red");
			ui.StepTwo_horizontalFrontDriveBox_center_y_Edit->setStyleSheet("color:red");
			ui.StepTwo_horizontalFrontDriveBox_center_z_Edit->setStyleSheet("color:red");

			ui.StepTwo_horizontalFrontDriveBox_normal_x_Edit->setText("NULL");
			ui.StepTwo_horizontalFrontDriveBox_normal_y_Edit->setText("NULL");
			ui.StepTwo_horizontalFrontDriveBox_normal_z_Edit->setText("NULL");

			ui.StepTwo_horizontalFrontDriveBox_center_x_Edit->setText("NULL");
			ui.StepTwo_horizontalFrontDriveBox_center_y_Edit->setText("NULL");
			ui.StepTwo_horizontalFrontDriveBox_center_z_Edit->setText("NULL");
			rightProcessingStatus = false;
			//添加到发送信息
			resultMessage.rightNormal.A = "NULL";
			resultMessage.rightNormal.B = "NULL";
			resultMessage.rightNormal.C = "NULL";
			resultMessage.rightCenter.X = "NULL";
			resultMessage.rightCenter.Y = "NULL";
			resultMessage.rightCenter.Z = "NULL";
		}
		

	}
	if (num == 203){
		leftFlangeHoleCenterResult = centerResult;
		leftFlangeHoleCircle_cloudPoints = borderCloud_ptr;
		if (rightCenterResult.status == true){
			//输出结果
			ui.StepTwo_horizontalEngine_holeCenter_x_Edit->setStyleSheet("color:black");
			ui.StepTwo_horizontalEngine_holeCenter_y_Edit->setStyleSheet("color:black");
			ui.StepTwo_horizontalEngine_holeCenter_z_Edit->setStyleSheet("color:black");

			ui.StepTwo_horizontalEngine_holeCenter_x_Edit->setText(QString::number(centerResult.center.x));
			ui.StepTwo_horizontalEngine_holeCenter_y_Edit->setText(QString::number(centerResult.center.y));
			ui.StepTwo_horizontalEngine_holeCenter_z_Edit->setText(QString::number(centerResult.center.z));

			leftProcessingStatus = true;
			//添加到发送信息
			resultMessage.leftCenter.X = QString::number(centerResult.center.x);
			resultMessage.leftCenter.Y = QString::number(centerResult.center.y);
			resultMessage.leftCenter.Z = QString::number(centerResult.center.z);

		}
		else{
			statusUpdate(QStringLiteral("右侧圆心拟合失败，重新采集！"), 1);
			ui.StepTwo_horizontalEngine_holeCenter_x_Edit->setStyleSheet("color:red");
			ui.StepTwo_horizontalEngine_holeCenter_y_Edit->setStyleSheet("color:red");
			ui.StepTwo_horizontalEngine_holeCenter_z_Edit->setStyleSheet("color:red");

			ui.StepTwo_horizontalEngine_holeCenter_x_Edit->setText("NULL");
			ui.StepTwo_horizontalEngine_holeCenter_y_Edit->setText("NULL");
			ui.StepTwo_horizontalEngine_holeCenter_z_Edit->setText("NULL");

			leftProcessingStatus = false;
			//添加到发送信息
			resultMessage.leftCenter.X = "NULL";
			resultMessage.leftCenter.Y = "NULL";
			resultMessage.leftCenter.Z = "NULL";
		}

	}
	if (num == 204){
		rightFlangeHoleCenterResult = centerResult;
		rightFlangeHoleCircle_cloudPoints = borderCloud_ptr;
		if (rightCenterResult.status == true){
			//输出结果
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_x_Edit->setStyleSheet("color:black");
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_y_Edit->setStyleSheet("color:black");
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_z_Edit->setStyleSheet("color:black");

			ui.StepTwo_horizontalFrontDriveBox_holeCenter_x_Edit->setText(QString::number(centerResult.center.x));
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_y_Edit->setText(QString::number(centerResult.center.y));
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_z_Edit->setText(QString::number(centerResult.center.z));
			rightProcessingStatus = true;
			//添加到发送信息
			resultMessage.rightCenter.X = QString::number(centerResult.center.x);
			resultMessage.rightCenter.Y = QString::number(centerResult.center.y);
			resultMessage.rightCenter.Z = QString::number(centerResult.center.z);

		}
		else{
			statusUpdate(QStringLiteral("右侧圆心拟合失败，重新采集！"), 1);
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_x_Edit->setStyleSheet("color:red");
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_y_Edit->setStyleSheet("color:red");
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_z_Edit->setStyleSheet("color:red");
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_x_Edit->setText("NULL");
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_y_Edit->setText("NULL");
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_z_Edit->setText("NULL");
			rightProcessingStatus = false;
			//添加到发送信息
			resultMessage.rightCenter.X = "NULL";
			resultMessage.rightCenter.Y = "NULL";
			resultMessage.rightCenter.Z = "NULL";
		}

	}
	if (num == 301){
		leftCenterResult = centerResult;
		leftCircle_cloudPoints = borderCloud_ptr;
		leftPlaneNormal = planeparam;
		if (planeparamStatus == true)
		{
			if (leftCenterResult.status == true){
				//输出结果
				ui.StepThree_horizontalSyntheticalDriveBox_normal_x_Edit->setStyleSheet("color:black");
				ui.StepThree_horizontalSyntheticalDriveBox_normal_y_Edit->setStyleSheet("color:black");
				ui.StepThree_horizontalSyntheticalDriveBox_normal_z_Edit->setStyleSheet("color:black");
				ui.StepThree_horizontalSyntheticalDriveBox_center_x_Edit->setStyleSheet("color:black");
				ui.StepThree_horizontalSyntheticalDriveBox_center_y_Edit->setStyleSheet("color:black");
				ui.StepThree_horizontalSyntheticalDriveBox_center_z_Edit->setStyleSheet("color:black");

				ui.StepThree_horizontalSyntheticalDriveBox_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepThree_horizontalSyntheticalDriveBox_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepThree_horizontalSyntheticalDriveBox_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepThree_horizontalSyntheticalDriveBox_center_x_Edit->setText(QString::number(centerResult.center.x));
				ui.StepThree_horizontalSyntheticalDriveBox_center_y_Edit->setText(QString::number(centerResult.center.y));
				ui.StepThree_horizontalSyntheticalDriveBox_center_z_Edit->setText(QString::number(centerResult.center.z));

				leftProcessingStatus = true;
				//添加到发送信息
				resultMessage.leftNormal.A = QString::number(planeparam.A);
				resultMessage.leftNormal.B = QString::number(planeparam.B);
				resultMessage.leftNormal.C = QString::number(planeparam.C);
				resultMessage.leftCenter.X = QString::number(centerResult.center.x);
				resultMessage.leftCenter.Y = QString::number(centerResult.center.y);
				resultMessage.leftCenter.Z = QString::number(centerResult.center.z);

			}
			else{
				statusUpdate(QStringLiteral("左侧圆心拟合失败，重新采集！"), 1);
				ui.StepThree_horizontalSyntheticalDriveBox_normal_x_Edit->setStyleSheet("color:black");
				ui.StepThree_horizontalSyntheticalDriveBox_normal_y_Edit->setStyleSheet("color:black");
				ui.StepThree_horizontalSyntheticalDriveBox_normal_z_Edit->setStyleSheet("color:black");
				ui.StepThree_horizontalSyntheticalDriveBox_center_x_Edit->setStyleSheet("color:red");
				ui.StepThree_horizontalSyntheticalDriveBox_center_y_Edit->setStyleSheet("color:red");
				ui.StepThree_horizontalSyntheticalDriveBox_center_z_Edit->setStyleSheet("color:red");

				ui.StepThree_horizontalSyntheticalDriveBox_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepThree_horizontalSyntheticalDriveBox_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepThree_horizontalSyntheticalDriveBox_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepThree_horizontalSyntheticalDriveBox_center_x_Edit->setText("NULL");
				ui.StepThree_horizontalSyntheticalDriveBox_center_y_Edit->setText("NULL");
				ui.StepThree_horizontalSyntheticalDriveBox_center_z_Edit->setText("NULL");

				leftProcessingStatus = true;
				//添加到发送信息
				resultMessage.leftNormal.A = QString::number(planeparam.A);
				resultMessage.leftNormal.B = QString::number(planeparam.B);
				resultMessage.leftNormal.C = QString::number(planeparam.C);
				resultMessage.leftCenter.X = "NULL";
				resultMessage.leftCenter.Y = "NULL";
				resultMessage.leftCenter.Z = "NULL";

			}
		}
		else
		{
			statusUpdate(QStringLiteral("左侧采集出现问题，请调整位置重新采集！"), 1);
			ui.StepThree_horizontalSyntheticalDriveBox_normal_x_Edit->setStyleSheet("color:red");
			ui.StepThree_horizontalSyntheticalDriveBox_normal_y_Edit->setStyleSheet("color:red");
			ui.StepThree_horizontalSyntheticalDriveBox_normal_z_Edit->setStyleSheet("color:red");
			ui.StepThree_horizontalSyntheticalDriveBox_center_x_Edit->setStyleSheet("color:red");
			ui.StepThree_horizontalSyntheticalDriveBox_center_y_Edit->setStyleSheet("color:red");
			ui.StepThree_horizontalSyntheticalDriveBox_center_z_Edit->setStyleSheet("color:red");

			ui.StepThree_horizontalSyntheticalDriveBox_normal_x_Edit->setText("NULL");
			ui.StepThree_horizontalSyntheticalDriveBox_normal_y_Edit->setText("NULL");
			ui.StepThree_horizontalSyntheticalDriveBox_normal_z_Edit->setText("NULL");

			ui.StepThree_horizontalSyntheticalDriveBox_center_x_Edit->setText("NULL");
			ui.StepThree_horizontalSyntheticalDriveBox_center_y_Edit->setText("NULL");
			ui.StepThree_horizontalSyntheticalDriveBox_center_z_Edit->setText("NULL");
			leftProcessingStatus = false;
			//添加到发送信息
			resultMessage.leftNormal.A = "NULL";
			resultMessage.leftNormal.B = "NULL";
			resultMessage.leftNormal.C = "NULL";
			resultMessage.leftCenter.X = "NULL";
			resultMessage.leftCenter.Y = "NULL";
			resultMessage.leftCenter.Z = "NULL";

		}

	}
	if (num == 302){
		rightCenterResult = centerResult;
		rightCircle_cloudPoints = borderCloud_ptr;
		rightPlaneNormal = planeparam;
		if (planeparamStatus == true)
		{
			if (rightCenterResult.status == true){
				//输出结果
				ui.StepThree_horizontalFrontDriveBox_normal_x_Edit->setStyleSheet("color:black");
				ui.StepThree_horizontalFrontDriveBox_normal_y_Edit->setStyleSheet("color:black");
				ui.StepThree_horizontalFrontDriveBox_normal_z_Edit->setStyleSheet("color:black");
				ui.StepThree_horizontalFrontDriveBox_center_x_Edit->setStyleSheet("color:black");
				ui.StepThree_horizontalFrontDriveBox_center_y_Edit->setStyleSheet("color:black");
				ui.StepThree_horizontalFrontDriveBox_center_z_Edit->setStyleSheet("color:black");

				ui.StepThree_horizontalFrontDriveBox_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepThree_horizontalFrontDriveBox_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepThree_horizontalFrontDriveBox_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepThree_horizontalFrontDriveBox_center_x_Edit->setText(QString::number(centerResult.center.x));
				ui.StepThree_horizontalFrontDriveBox_center_y_Edit->setText(QString::number(centerResult.center.y));
				ui.StepThree_horizontalFrontDriveBox_center_z_Edit->setText(QString::number(centerResult.center.z));

				rightProcessingStatus = true;
				//添加到发送信息
				resultMessage.rightNormal.A = QString::number(planeparam.A);
				resultMessage.rightNormal.B = QString::number(planeparam.B);
				resultMessage.rightNormal.C = QString::number(planeparam.C);
				resultMessage.rightCenter.X = QString::number(centerResult.center.x);
				resultMessage.rightCenter.Y = QString::number(centerResult.center.y);
				resultMessage.rightCenter.Z = QString::number(centerResult.center.z);

			}
			else{
				statusUpdate(QStringLiteral("右侧圆心拟合失败，重新采集！"), 1);
				ui.StepThree_horizontalFrontDriveBox_normal_x_Edit->setStyleSheet("color:black");
				ui.StepThree_horizontalFrontDriveBox_normal_y_Edit->setStyleSheet("color:black");
				ui.StepThree_horizontalFrontDriveBox_normal_z_Edit->setStyleSheet("color:black");
				ui.StepThree_horizontalFrontDriveBox_center_x_Edit->setStyleSheet("color:red");
				ui.StepThree_horizontalFrontDriveBox_center_y_Edit->setStyleSheet("color:red");
				ui.StepThree_horizontalFrontDriveBox_center_z_Edit->setStyleSheet("color:red");

				ui.StepThree_horizontalFrontDriveBox_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepThree_horizontalFrontDriveBox_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepThree_horizontalFrontDriveBox_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepThree_horizontalFrontDriveBox_center_x_Edit->setText("NULL");
				ui.StepThree_horizontalFrontDriveBox_center_y_Edit->setText("NULL");
				ui.StepThree_horizontalFrontDriveBox_center_z_Edit->setText("NULL");

				rightProcessingStatus = true;


			}
		}
		else
		{
			statusUpdate(QStringLiteral("右侧采集出现问题，请调整位置重新采集！"), 1);
			ui.StepThree_horizontalFrontDriveBox_normal_x_Edit->setStyleSheet("color:red");
			ui.StepThree_horizontalFrontDriveBox_normal_y_Edit->setStyleSheet("color:red");
			ui.StepThree_horizontalFrontDriveBox_normal_z_Edit->setStyleSheet("color:red");
			ui.StepThree_horizontalFrontDriveBox_center_x_Edit->setStyleSheet("color:red");
			ui.StepThree_horizontalFrontDriveBox_center_y_Edit->setStyleSheet("color:red");
			ui.StepThree_horizontalFrontDriveBox_center_z_Edit->setStyleSheet("color:red");

			ui.StepThree_horizontalFrontDriveBox_normal_x_Edit->setText("NULL");
			ui.StepThree_horizontalFrontDriveBox_normal_y_Edit->setText("NULL");
			ui.StepThree_horizontalFrontDriveBox_normal_z_Edit->setText("NULL");

			ui.StepThree_horizontalFrontDriveBox_center_x_Edit->setText("NULL");
			ui.StepThree_horizontalFrontDriveBox_center_y_Edit->setText("NULL");
			ui.StepThree_horizontalFrontDriveBox_center_z_Edit->setText("NULL");
			rightProcessingStatus = false;
			//添加到发送信息
			resultMessage.rightNormal.A = "NULL";
			resultMessage.rightNormal.B = "NULL";
			resultMessage.rightNormal.C = "NULL";
			resultMessage.rightCenter.X = "NULL";
			resultMessage.rightCenter.Y = "NULL";
			resultMessage.rightCenter.Z = "NULL";
		}

	}
	if (num == 303){
		leftFlangeHoleCenterResult = centerResult;
		leftFlangeHoleCircle_cloudPoints = borderCloud_ptr;
		if (rightCenterResult.status == true){
			//输出结果
			ui.StepThree_horizontalSyntheticalDriveBox_holeCenter_x_Edit->setStyleSheet("color:black");
			ui.StepThree_horizontalSyntheticalDriveBox_holeCenter_y_Edit->setStyleSheet("color:black");
			ui.StepThree_horizontalSyntheticalDriveBox_holeCenter_z_Edit->setStyleSheet("color:black");

			ui.StepThree_horizontalSyntheticalDriveBox_holeCenter_x_Edit->setText(QString::number(centerResult.center.x));
			ui.StepThree_horizontalSyntheticalDriveBox_holeCenter_y_Edit->setText(QString::number(centerResult.center.y));
			ui.StepThree_horizontalSyntheticalDriveBox_holeCenter_z_Edit->setText(QString::number(centerResult.center.z));

			leftProcessingStatus = true;
			//添加到发送信息
			resultMessage.leftCenter.X = QString::number(centerResult.center.x);
			resultMessage.leftCenter.Y = QString::number(centerResult.center.y);
			resultMessage.leftCenter.Z = QString::number(centerResult.center.z);

		}
		else{
			statusUpdate(QStringLiteral("左侧圆心拟合失败，重新采集！"), 1);
			ui.StepThree_horizontalSyntheticalDriveBox_holeCenter_x_Edit->setStyleSheet("color:red");
			ui.StepThree_horizontalSyntheticalDriveBox_holeCenter_y_Edit->setStyleSheet("color:red");
			ui.StepThree_horizontalSyntheticalDriveBox_holeCenter_z_Edit->setStyleSheet("color:red");

			ui.StepThree_horizontalSyntheticalDriveBox_holeCenter_x_Edit->setText("NULL");
			ui.StepThree_horizontalSyntheticalDriveBox_holeCenter_y_Edit->setText("NULL");
			ui.StepThree_horizontalSyntheticalDriveBox_holeCenter_z_Edit->setText("NULL");

			leftProcessingStatus = false;
			//添加到发送信息
			resultMessage.leftCenter.X = "NULL";
			resultMessage.leftCenter.Y = "NULL";
			resultMessage.leftCenter.Z = "NULL";
		}

	}
	if (num == 304){
		rightFlangeHoleCenterResult = centerResult;
		rightFlangeHoleCircle_cloudPoints = borderCloud_ptr;
		if (rightCenterResult.status == true){
			//输出结果
			ui.StepThree_horizontalFrontDriveBox_holeCenter_x_Edit->setStyleSheet("color:black");
			ui.StepThree_horizontalFrontDriveBox_holeCenter_y_Edit->setStyleSheet("color:black");
			ui.StepThree_horizontalFrontDriveBox_holeCenter_z_Edit->setStyleSheet("color:black");

			ui.StepThree_horizontalFrontDriveBox_holeCenter_x_Edit->setText(QString::number(centerResult.center.x));
			ui.StepThree_horizontalFrontDriveBox_holeCenter_y_Edit->setText(QString::number(centerResult.center.y));
			ui.StepThree_horizontalFrontDriveBox_holeCenter_z_Edit->setText(QString::number(centerResult.center.z));

			rightProcessingStatus = true;
			//添加到发送信息
			resultMessage.rightCenter.X = QString::number(centerResult.center.x);
			resultMessage.rightCenter.Y = QString::number(centerResult.center.y);
			resultMessage.rightCenter.Z = QString::number(centerResult.center.z);

		}
		else{
			statusUpdate(QStringLiteral("右侧圆心拟合失败，重新采集！"), 1);
			ui.StepThree_horizontalFrontDriveBox_holeCenter_x_Edit->setStyleSheet("color:red");
			ui.StepThree_horizontalFrontDriveBox_holeCenter_y_Edit->setStyleSheet("color:red");
			ui.StepThree_horizontalFrontDriveBox_holeCenter_z_Edit->setStyleSheet("color:red");

			ui.StepThree_horizontalFrontDriveBox_holeCenter_x_Edit->setText(QString::number(centerResult.center.x));
			ui.StepThree_horizontalFrontDriveBox_holeCenter_y_Edit->setText(QString::number(centerResult.center.y));
			ui.StepThree_horizontalFrontDriveBox_holeCenter_z_Edit->setText(QString::number(centerResult.center.z));

			rightProcessingStatus = false;
			//添加到发送信息
			resultMessage.rightCenter.X = "NULL";
			resultMessage.rightCenter.Y = "NULL";
			resultMessage.rightCenter.Z = "NULL";
		}

	}
	if (num == 401){
		leftCenterResult = centerResult;
		leftCircle_cloudPoints = borderCloud_ptr;
		leftPlaneNormal = planeparam;
		if (planeparamStatus == true)
		{
			if (leftCenterResult.status == true){
				//输出结果
				ui.StepFour_horizontalSyntheticalDriveBox_normal_x_Edit->setStyleSheet("color:black");
				ui.StepFour_horizontalSyntheticalDriveBox_normal_y_Edit->setStyleSheet("color:black");
				ui.StepFour_horizontalSyntheticalDriveBox_normal_z_Edit->setStyleSheet("color:black");
				ui.StepFour_horizontalSyntheticalDriveBox_center_x_Edit->setStyleSheet("color:black");
				ui.StepFour_horizontalSyntheticalDriveBox_center_y_Edit->setStyleSheet("color:black");
				ui.StepFour_horizontalSyntheticalDriveBox_center_z_Edit->setStyleSheet("color:black");

				ui.StepFour_horizontalSyntheticalDriveBox_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepFour_horizontalSyntheticalDriveBox_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepFour_horizontalSyntheticalDriveBox_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepFour_horizontalSyntheticalDriveBox_center_x_Edit->setText(QString::number(centerResult.center.x));
				ui.StepFour_horizontalSyntheticalDriveBox_center_y_Edit->setText(QString::number(centerResult.center.y));
				ui.StepFour_horizontalSyntheticalDriveBox_center_z_Edit->setText(QString::number(centerResult.center.z));

				leftProcessingStatus = true;
				//添加到发送信息
				resultMessage.leftNormal.A = QString::number(planeparam.A);
				resultMessage.leftNormal.B = QString::number(planeparam.B);
				resultMessage.leftNormal.C = QString::number(planeparam.C);
				resultMessage.leftCenter.X = QString::number(centerResult.center.x);
				resultMessage.leftCenter.Y = QString::number(centerResult.center.y);
				resultMessage.leftCenter.Z = QString::number(centerResult.center.z);

			}
			else{
				statusUpdate(QStringLiteral("左侧圆心拟合失败，重新采集！"), 1);
				ui.StepFour_horizontalSyntheticalDriveBox_normal_x_Edit->setStyleSheet("color:black");
				ui.StepFour_horizontalSyntheticalDriveBox_normal_y_Edit->setStyleSheet("color:black");
				ui.StepFour_horizontalSyntheticalDriveBox_normal_z_Edit->setStyleSheet("color:black");
				ui.StepFour_horizontalSyntheticalDriveBox_center_x_Edit->setStyleSheet("color:red");
				ui.StepFour_horizontalSyntheticalDriveBox_center_y_Edit->setStyleSheet("color:red");
				ui.StepFour_horizontalSyntheticalDriveBox_center_z_Edit->setStyleSheet("color:red");
				ui.StepFour_horizontalSyntheticalDriveBox_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepFour_horizontalSyntheticalDriveBox_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepFour_horizontalSyntheticalDriveBox_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepFour_horizontalSyntheticalDriveBox_center_x_Edit->setText("NULL");
				ui.StepFour_horizontalSyntheticalDriveBox_center_y_Edit->setText("NULL");
				ui.StepFour_horizontalSyntheticalDriveBox_center_z_Edit->setText("NULL");

				leftProcessingStatus = true;
				//添加到发送信息
				resultMessage.leftNormal.A = QString::number(planeparam.A);
				resultMessage.leftNormal.B = QString::number(planeparam.B);
				resultMessage.leftNormal.C = QString::number(planeparam.C);
				resultMessage.leftCenter.X = "NULL";
				resultMessage.leftCenter.Y = "NULL";
				resultMessage.leftCenter.Z = "NULL";

			}
		}
		else
		{
			statusUpdate(QStringLiteral("采集出现问题，请调整位置重新采集！"), 1);
			ui.StepFour_horizontalSyntheticalDriveBox_normal_x_Edit->setStyleSheet("color:red");
			ui.StepFour_horizontalSyntheticalDriveBox_normal_y_Edit->setStyleSheet("color:red");
			ui.StepFour_horizontalSyntheticalDriveBox_normal_z_Edit->setStyleSheet("color:red");
			ui.StepFour_horizontalSyntheticalDriveBox_center_x_Edit->setStyleSheet("color:red");
			ui.StepFour_horizontalSyntheticalDriveBox_center_y_Edit->setStyleSheet("color:red");
			ui.StepFour_horizontalSyntheticalDriveBox_center_z_Edit->setStyleSheet("color:red");
			ui.StepFour_horizontalSyntheticalDriveBox_normal_x_Edit->setText("NULL");
			ui.StepFour_horizontalSyntheticalDriveBox_normal_y_Edit->setText("NULL");
			ui.StepFour_horizontalSyntheticalDriveBox_normal_z_Edit->setText("NULL");

			ui.StepFour_horizontalSyntheticalDriveBox_center_x_Edit->setText("NULL");
			ui.StepFour_horizontalSyntheticalDriveBox_center_y_Edit->setText("NULL");
			ui.StepFour_horizontalSyntheticalDriveBox_center_z_Edit->setText("NULL");

			leftProcessingStatus = true;
			//添加到发送信息
			resultMessage.leftNormal.A = "NULL";
			resultMessage.leftNormal.B = "NULL";
			resultMessage.leftNormal.C = "NULL";
			resultMessage.leftCenter.X = "NULL";
			resultMessage.leftCenter.Y = "NULL";
			resultMessage.leftCenter.Z = "NULL";
		}

	}
	if (num == 402){
		rightCenterResult = centerResult;
		rightCircle_cloudPoints = borderCloud_ptr;
		rightPlaneNormal = planeparam;
		if (planeparamStatus == true)
		{
			if (rightCenterResult.status == true){
				//输出结果
				ui.StepFour_horizontalFrontDriveBox_normal_x_Edit->setStyleSheet("color:black");
				ui.StepFour_horizontalFrontDriveBox_normal_y_Edit->setStyleSheet("color:black");
				ui.StepFour_horizontalFrontDriveBox_normal_z_Edit->setStyleSheet("color:black");
				ui.StepFour_horizontalFrontDriveBox_center_x_Edit->setStyleSheet("color:black");
				ui.StepFour_horizontalFrontDriveBox_center_y_Edit->setStyleSheet("color:black");
				ui.StepFour_horizontalFrontDriveBox_center_z_Edit->setStyleSheet("color:black");

				ui.StepFour_horizontalFrontDriveBox_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepFour_horizontalFrontDriveBox_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepFour_horizontalFrontDriveBox_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepFour_horizontalFrontDriveBox_center_x_Edit->setText(QString::number(centerResult.center.x));
				ui.StepFour_horizontalFrontDriveBox_center_y_Edit->setText(QString::number(centerResult.center.y));
				ui.StepFour_horizontalFrontDriveBox_center_z_Edit->setText(QString::number(centerResult.center.z));

				rightProcessingStatus = true;
				//添加到发送信息
				resultMessage.rightNormal.A = QString::number(planeparam.A);
				resultMessage.rightNormal.B = QString::number(planeparam.B);
				resultMessage.rightNormal.C = QString::number(planeparam.C);
				resultMessage.rightCenter.X = QString::number(centerResult.center.x);
				resultMessage.rightCenter.Y = QString::number(centerResult.center.y);
				resultMessage.rightCenter.Z = QString::number(centerResult.center.z);
			}
			else{
				statusUpdate(QStringLiteral("右侧圆心拟合误差过大，重新采集！"), 1);
				ui.StepFour_horizontalFrontDriveBox_normal_x_Edit->setStyleSheet("color:black");
				ui.StepFour_horizontalFrontDriveBox_normal_y_Edit->setStyleSheet("color:black");
				ui.StepFour_horizontalFrontDriveBox_normal_z_Edit->setStyleSheet("color:black");
				ui.StepFour_horizontalFrontDriveBox_center_x_Edit->setStyleSheet("color:red");
				ui.StepFour_horizontalFrontDriveBox_center_y_Edit->setStyleSheet("color:red");
				ui.StepFour_horizontalFrontDriveBox_center_z_Edit->setStyleSheet("color:red");
				ui.StepFour_horizontalFrontDriveBox_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepFour_horizontalFrontDriveBox_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepFour_horizontalFrontDriveBox_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepFour_horizontalFrontDriveBox_center_x_Edit->setText("NULL");
				ui.StepFour_horizontalFrontDriveBox_center_y_Edit->setText("NULL");
				ui.StepFour_horizontalFrontDriveBox_center_z_Edit->setText("NULL");

				rightProcessingStatus = true;
				//添加到发送信息
				resultMessage.rightNormal.A = QString::number(planeparam.A);
				resultMessage.rightNormal.B = QString::number(planeparam.B);
				resultMessage.rightNormal.C = QString::number(planeparam.C);
				resultMessage.rightCenter.X = "NULL";
				resultMessage.rightCenter.Y = "NULL";
				resultMessage.rightCenter.Z = "NULL";
			}
		}
		else
		{
			statusUpdate(QStringLiteral("右侧采集出现问题，请调整位置重新采集！"), 1);
			ui.StepFour_horizontalFrontDriveBox_normal_x_Edit->setStyleSheet("color:red");
			ui.StepFour_horizontalFrontDriveBox_normal_y_Edit->setStyleSheet("color:red");
			ui.StepFour_horizontalFrontDriveBox_normal_z_Edit->setStyleSheet("color:red");
			ui.StepFour_horizontalFrontDriveBox_center_x_Edit->setStyleSheet("color:red");
			ui.StepFour_horizontalFrontDriveBox_center_y_Edit->setStyleSheet("color:red");
			ui.StepFour_horizontalFrontDriveBox_center_z_Edit->setStyleSheet("color:red");
			ui.StepFour_horizontalFrontDriveBox_normal_x_Edit->setText("NULL");
			ui.StepFour_horizontalFrontDriveBox_normal_y_Edit->setText("NULL");
			ui.StepFour_horizontalFrontDriveBox_normal_z_Edit->setText("NULL");

			ui.StepFour_horizontalFrontDriveBox_center_x_Edit->setText("NULL");
			ui.StepFour_horizontalFrontDriveBox_center_y_Edit->setText("NULL");
			ui.StepFour_horizontalFrontDriveBox_center_z_Edit->setText("NULL");
			rightProcessingStatus = false;
			//添加到发送信息
			resultMessage.rightNormal.A = "NULL";
			resultMessage.rightNormal.B = "NULL";
			resultMessage.rightNormal.C = "NULL";
			resultMessage.rightCenter.X = "NULL";
			resultMessage.rightCenter.Y = "NULL";
			resultMessage.rightCenter.Z = "NULL";

		}
	}
	if (num == 501){
		leftCenterResult = centerResult;
		leftCircle_cloudPoints = borderCloud_ptr;
		leftPlaneNormal = planeparam;
		if (planeparamStatus == true)
		{
			if (leftCenterResult.status == true){
				//输出结果
				ui.StepFive_horizontalEngine_normal_x_Edit->setStyleSheet("color:black");
				ui.StepFive_horizontalEngine_normal_y_Edit->setStyleSheet("color:black");
				ui.StepFive_horizontalEngine_normal_z_Edit->setStyleSheet("color:black");
				ui.StepFive_horizontalEngine_center_x_Edit->setStyleSheet("color:black");
				ui.StepFive_horizontalEngine_center_y_Edit->setStyleSheet("color:black");
				ui.StepFive_horizontalEngine_center_z_Edit->setStyleSheet("color:black");

				ui.StepFive_horizontalEngine_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepFive_horizontalEngine_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepFive_horizontalEngine_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepFive_horizontalEngine_center_x_Edit->setText(QString::number(centerResult.center.x));
				ui.StepFive_horizontalEngine_center_y_Edit->setText(QString::number(centerResult.center.y));
				ui.StepFive_horizontalEngine_center_z_Edit->setText(QString::number(centerResult.center.z));

				leftProcessingStatus = true;
				//添加到发送信息
				resultMessage.leftNormal.A = QString::number(planeparam.A);
				resultMessage.leftNormal.B = QString::number(planeparam.B);
				resultMessage.leftNormal.C = QString::number(planeparam.C);
				resultMessage.leftCenter.X = QString::number(centerResult.center.x);
				resultMessage.leftCenter.Y = QString::number(centerResult.center.y);
				resultMessage.leftCenter.Z = QString::number(centerResult.center.z);

			}
			else{
				statusUpdate(QStringLiteral("左侧圆心拟合失败，重新采集！"), 1);
				ui.StepFive_horizontalEngine_normal_x_Edit->setStyleSheet("color:black");
				ui.StepFive_horizontalEngine_normal_y_Edit->setStyleSheet("color:black");
				ui.StepFive_horizontalEngine_normal_z_Edit->setStyleSheet("color:black");
				ui.StepFive_horizontalEngine_center_x_Edit->setStyleSheet("color:red");
				ui.StepFive_horizontalEngine_center_y_Edit->setStyleSheet("color:red");
				ui.StepFive_horizontalEngine_center_z_Edit->setStyleSheet("color:red");

				ui.StepFive_horizontalEngine_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepFive_horizontalEngine_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepFive_horizontalEngine_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepFive_horizontalEngine_center_x_Edit->setText("NULL");
				ui.StepFive_horizontalEngine_center_y_Edit->setText("NULL");
				ui.StepFive_horizontalEngine_center_z_Edit->setText("NULL");

				leftProcessingStatus = true;
				//添加到发送信息
				resultMessage.leftNormal.A = QString::number(planeparam.A);
				resultMessage.leftNormal.B = QString::number(planeparam.B);
				resultMessage.leftNormal.C = QString::number(planeparam.C);
				resultMessage.leftCenter.X = "NULL";
				resultMessage.leftCenter.Y = "NULL";
				resultMessage.leftCenter.Z = "NULL";
			}
		}
		else
		{
			
			statusUpdate(QStringLiteral("左侧采集出现问题，请调整位置重新采集！"), 1); 
			ui.StepFive_horizontalEngine_normal_x_Edit->setStyleSheet("color:red");
			ui.StepFive_horizontalEngine_normal_y_Edit->setStyleSheet("color:red");
			ui.StepFive_horizontalEngine_normal_z_Edit->setStyleSheet("color:red");
			ui.StepFive_horizontalEngine_center_x_Edit->setStyleSheet("color:red");
			ui.StepFive_horizontalEngine_center_y_Edit->setStyleSheet("color:red");
			ui.StepFive_horizontalEngine_center_z_Edit->setStyleSheet("color:red");
			ui.StepFive_horizontalEngine_normal_x_Edit->setText("NULL");
			ui.StepFive_horizontalEngine_normal_y_Edit->setText("NULL");
			ui.StepFive_horizontalEngine_normal_z_Edit->setText("NULL");

			ui.StepFive_horizontalEngine_center_x_Edit->setText("NULL");
			ui.StepFive_horizontalEngine_center_y_Edit->setText("NULL");
			ui.StepFive_horizontalEngine_center_z_Edit->setText("NULL");

			leftProcessingStatus = false;
			//添加到发送信息
			resultMessage.leftNormal.A = "NULL";
			resultMessage.leftNormal.B = "NULL";
			resultMessage.leftNormal.C = "NULL";
			resultMessage.leftCenter.X = "NULL";
			resultMessage.leftCenter.Y = "NULL";
			resultMessage.leftCenter.Z = "NULL";
		}
		

	}
	if (num == 502)
	{
		rightCenterResult = centerResult;
		rightCircle_cloudPoints = borderCloud_ptr;
		rightPlaneNormal = planeparam;
		if (planeparamStatus == true)
		{
			if (rightCenterResult.status == true){
				ui.StepFive_horizontalFrontDriveBox_normal_x_Edit->setStyleSheet("color:black");
				ui.StepFive_horizontalFrontDriveBox_normal_y_Edit->setStyleSheet("color:black");
				ui.StepFive_horizontalFrontDriveBox_normal_z_Edit->setStyleSheet("color:black");
				ui.StepFive_horizontalFrontDriveBox_center_x_Edit->setStyleSheet("color:black");
				ui.StepFive_horizontalFrontDriveBox_center_y_Edit->setStyleSheet("color:black");
				ui.StepFive_horizontalFrontDriveBox_center_z_Edit->setStyleSheet("color:black");

				ui.StepFive_horizontalFrontDriveBox_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepFive_horizontalFrontDriveBox_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepFive_horizontalFrontDriveBox_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepFive_horizontalFrontDriveBox_center_x_Edit->setText(QString::number(centerResult.center.x));
				ui.StepFive_horizontalFrontDriveBox_center_y_Edit->setText(QString::number(centerResult.center.y));
				ui.StepFive_horizontalFrontDriveBox_center_z_Edit->setText(QString::number(centerResult.center.z));

				rightProcessingStatus = true;
				//添加到发送信息
				resultMessage.rightNormal.A = QString::number(planeparam.A);
				resultMessage.rightNormal.B = QString::number(planeparam.B);
				resultMessage.rightNormal.C = QString::number(planeparam.C);
				resultMessage.rightCenter.X = QString::number(centerResult.center.x);
				resultMessage.rightCenter.Y = QString::number(centerResult.center.y);
				resultMessage.rightCenter.Z = QString::number(centerResult.center.z);

			}
			else{
				statusUpdate(QStringLiteral("右侧圆心拟合失败，重新采集！"), 1);
				ui.StepFive_horizontalFrontDriveBox_normal_x_Edit->setStyleSheet("color:black");
				ui.StepFive_horizontalFrontDriveBox_normal_y_Edit->setStyleSheet("color:black");
				ui.StepFive_horizontalFrontDriveBox_normal_z_Edit->setStyleSheet("color:black");
				ui.StepFive_horizontalFrontDriveBox_center_x_Edit->setStyleSheet("color:red");
				ui.StepFive_horizontalFrontDriveBox_center_y_Edit->setStyleSheet("color:red");
				ui.StepFive_horizontalFrontDriveBox_center_z_Edit->setStyleSheet("color:red");
				ui.StepFive_horizontalFrontDriveBox_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepFive_horizontalFrontDriveBox_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepFive_horizontalFrontDriveBox_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepFive_horizontalFrontDriveBox_center_x_Edit->setText("NULL");
				ui.StepFive_horizontalFrontDriveBox_center_y_Edit->setText("NULL");
				ui.StepFive_horizontalFrontDriveBox_center_z_Edit->setText("NULL");

				rightProcessingStatus = true;
				//添加到发送信息
				resultMessage.rightNormal.A = QString::number(planeparam.A);
				resultMessage.rightNormal.B = QString::number(planeparam.B);
				resultMessage.rightNormal.C = QString::number(planeparam.C);
				resultMessage.rightCenter.X = "NULL";
				resultMessage.rightCenter.Y = "NULL";
				resultMessage.rightCenter.Z = "NULL";
			}
		}
		else
		{
			statusUpdate(QStringLiteral("右侧采集出现问题，请调整位置重新采集！"), 1);
			ui.StepFive_horizontalFrontDriveBox_normal_x_Edit->setStyleSheet("color:red");
			ui.StepFive_horizontalFrontDriveBox_normal_y_Edit->setStyleSheet("color:red");
			ui.StepFive_horizontalFrontDriveBox_normal_z_Edit->setStyleSheet("color:red");
			ui.StepFive_horizontalFrontDriveBox_center_x_Edit->setStyleSheet("color:red");
			ui.StepFive_horizontalFrontDriveBox_center_y_Edit->setStyleSheet("color:red");
			ui.StepFive_horizontalFrontDriveBox_center_z_Edit->setStyleSheet("color:red");
			ui.StepFive_horizontalFrontDriveBox_normal_x_Edit->setText("NULL");
			ui.StepFive_horizontalFrontDriveBox_normal_y_Edit->setText("NULL");
			ui.StepFive_horizontalFrontDriveBox_normal_z_Edit->setText("NULL");

			ui.StepFive_horizontalFrontDriveBox_center_x_Edit->setText("NULL");
			ui.StepFive_horizontalFrontDriveBox_center_y_Edit->setText("NULL");
			ui.StepFive_horizontalFrontDriveBox_center_z_Edit->setText("NULL");
			rightProcessingStatus = false;
			//添加到发送信息
			resultMessage.rightNormal.A = "NULL";
			resultMessage.rightNormal.B = "NULL";
			resultMessage.rightNormal.C = "NULL";
			resultMessage.rightCenter.X = "NULL";
			resultMessage.rightCenter.Y = "NULL";
			resultMessage.rightCenter.Z = "NULL";
		}
	}
		

	
	/**
	*纵置发动机
	*/
	if (num == 1010){
		if (planeparamStatus == true){
			//输出结果
			ui.StepOne_verticalEngine_normal_x_Edit->setStyleSheet("color:black");
			ui.StepOne_verticalEngine_normal_y_Edit->setStyleSheet("color:black");
			ui.StepOne_verticalEngine_normal_z_Edit->setStyleSheet("color:black");

			ui.StepOne_verticalEngine_normal_x_Edit->setText(QString::number(planeparam.A));
			ui.StepOne_verticalEngine_normal_y_Edit->setText(QString::number(planeparam.B));
			ui.StepOne_verticalEngine_normal_z_Edit->setText(QString::number(planeparam.C));
			
			leftPlaneNormal = planeparam;
			leftProcessingStatus = true;
			//添加到发送信息
			resultMessage.leftNormal.A = QString::number(planeparam.A);
			resultMessage.leftNormal.B = QString::number(planeparam.B);
			resultMessage.leftNormal.C = QString::number(planeparam.C);
		}
		else{
			statusUpdate(QStringLiteral("左侧采集出现问题，请调整位置重新采集！"), 1);
			ui.StepOne_verticalEngine_normal_x_Edit->setStyleSheet("color:red");
			ui.StepOne_verticalEngine_normal_y_Edit->setStyleSheet("color:red");
			ui.StepOne_verticalEngine_normal_z_Edit->setStyleSheet("color:red");
			ui.StepOne_verticalEngine_normal_x_Edit->setText("NULL");
			ui.StepOne_verticalEngine_normal_y_Edit->setText("NULL");
			ui.StepOne_verticalEngine_normal_z_Edit->setText("NULL");

			leftProcessingStatus = false;
			//添加到发送信息
			resultMessage.leftNormal.A = "NULL";
			resultMessage.leftNormal.B = "NULL";
			resultMessage.leftNormal.C = "NULL";
		}

	}
	if (num == 1020){
		transmissionCaseCenterResultFirst = centerResult;
		transmissionCaseCircleFirst_cloudPoints = borderCloud_ptr;
		if (transmissionCaseCenterResultFirst.status == true){
			//输出结果
			ui.StepOne_verticalSyntheticalDriveBox_First_center_x_Edit->setStyleSheet("color:black");
			ui.StepOne_verticalSyntheticalDriveBox_First_center_y_Edit->setStyleSheet("color:black");
			ui.StepOne_verticalSyntheticalDriveBox_First_center_z_Edit->setStyleSheet("color:black");
			ui.StepOne_verticalSyntheticalDriveBox_First_center_x_Edit->setText(QString::number(centerResult.center.x));
			ui.StepOne_verticalSyntheticalDriveBox_First_center_y_Edit->setText(QString::number(centerResult.center.y));
			ui.StepOne_verticalSyntheticalDriveBox_First_center_z_Edit->setText(QString::number(centerResult.center.z));

			rightProcessingStatus = true;
			//添加到发送信息
			resultMessage.rightCenter.X = QString::number(centerResult.center.x);
			resultMessage.rightCenter.Y = QString::number(centerResult.center.y);
			resultMessage.rightCenter.Z = QString::number(centerResult.center.z);
		}
		else{
			statusUpdate(QStringLiteral("右侧圆心拟合失败，重新采集！"), 1);
			ui.StepOne_verticalSyntheticalDriveBox_First_center_x_Edit->setStyleSheet("color:red");
			ui.StepOne_verticalSyntheticalDriveBox_First_center_y_Edit->setStyleSheet("color:red");
			ui.StepOne_verticalSyntheticalDriveBox_First_center_z_Edit->setStyleSheet("color:red");
			ui.StepOne_verticalSyntheticalDriveBox_First_center_x_Edit->setText(QString::number(centerResult.center.x));
			ui.StepOne_verticalSyntheticalDriveBox_First_center_y_Edit->setText(QString::number(centerResult.center.y));
			ui.StepOne_verticalSyntheticalDriveBox_First_center_z_Edit->setText(QString::number(centerResult.center.z));
			rightProcessingStatus = false;
			//添加到发送信息
			resultMessage.rightCenter.X = "NULL";
			resultMessage.rightCenter.Y = "NULL";
			resultMessage.rightCenter.Z = "NULL";
		}

	}
	if (num == 1040){
		transmissionCaseCenterResultSecond = centerResult;
		transmissionCaseCircleSecond_cloudPoints = borderCloud_ptr;
		if (transmissionCaseCenterResultSecond.status == true)
		{
			//计算方向矢量
			directionVector.x = transmissionCaseCenterResultFirst.center.x - transmissionCaseCenterResultSecond.center.x;
			directionVector.y = transmissionCaseCenterResultFirst.center.y - transmissionCaseCenterResultSecond.center.y;
			directionVector.z = transmissionCaseCenterResultFirst.center.z - transmissionCaseCenterResultSecond.center.z;
			QString fileName = engineNumFileName + "/outputFile/StepOne/syntheticalDriveBoxDirectionVector.csv";//写入文件的目录
			QString time = ReturnTime();
			QFile file(fileName);
			file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
			QTextStream out(&file);
			out << time << "," << directionVector.x << "," << directionVector.y << "," << directionVector.y << "\n";
			file.close();
			//输出结果
			ui.StepOne_verticalSyntheticalDriveBox_Second_center_x_Edit->setStyleSheet("color:black");
			ui.StepOne_verticalSyntheticalDriveBox_Second_center_y_Edit->setStyleSheet("color:black");
			ui.StepOne_verticalSyntheticalDriveBox_Second_center_z_Edit->setStyleSheet("color:black");
			ui.StepOne_verticalSyntheticalDriveBox_directionVector_x_Edit->setStyleSheet("color:black");
			ui.StepOne_verticalSyntheticalDriveBox_directionVector_y_Edit->setStyleSheet("color:black");
			ui.StepOne_verticalSyntheticalDriveBox_directionVector_z_Edit->setStyleSheet("color:black");

			ui.StepOne_verticalSyntheticalDriveBox_Second_center_x_Edit->setText(QString::number(centerResult.center.x));
			ui.StepOne_verticalSyntheticalDriveBox_Second_center_y_Edit->setText(QString::number(centerResult.center.y));
			ui.StepOne_verticalSyntheticalDriveBox_Second_center_z_Edit->setText(QString::number(centerResult.center.z));

			ui.StepOne_verticalSyntheticalDriveBox_directionVector_x_Edit->setText(QString::number(directionVector.x));
			ui.StepOne_verticalSyntheticalDriveBox_directionVector_y_Edit->setText(QString::number(directionVector.y));
			ui.StepOne_verticalSyntheticalDriveBox_directionVector_z_Edit->setText(QString::number(directionVector.z));

			rightProcessingStatus = true;
			//添加到发送信息
			resultMessage.rightCenter.X = QString::number(centerResult.center.x);
			resultMessage.rightCenter.Y = QString::number(centerResult.center.y);
			resultMessage.rightCenter.Z = QString::number(centerResult.center.z);
			resultMessage.directionVector.A = QString::number(directionVector.x);
			resultMessage.directionVector.B = QString::number(directionVector.y);
			resultMessage.directionVector.C = QString::number(directionVector.z);
		}
		else{
			statusUpdate(QStringLiteral("右侧圆心拟合失败，重新采集！"), 1);
			ui.StepOne_verticalSyntheticalDriveBox_Second_center_x_Edit->setStyleSheet("color:red");
			ui.StepOne_verticalSyntheticalDriveBox_Second_center_y_Edit->setStyleSheet("color:red");
			ui.StepOne_verticalSyntheticalDriveBox_Second_center_z_Edit->setStyleSheet("color:red");
			ui.StepOne_verticalSyntheticalDriveBox_directionVector_x_Edit->setStyleSheet("color:red");
			ui.StepOne_verticalSyntheticalDriveBox_directionVector_y_Edit->setStyleSheet("color:red");
			ui.StepOne_verticalSyntheticalDriveBox_directionVector_z_Edit->setStyleSheet("color:red");
			ui.StepOne_verticalSyntheticalDriveBox_Second_center_x_Edit->setText(QString::number(centerResult.center.x));
			ui.StepOne_verticalSyntheticalDriveBox_Second_center_y_Edit->setText(QString::number(centerResult.center.y));
			ui.StepOne_verticalSyntheticalDriveBox_Second_center_z_Edit->setText(QString::number(centerResult.center.z));

			ui.StepOne_verticalSyntheticalDriveBox_directionVector_x_Edit->setText(QString::number(directionVector.x));
			ui.StepOne_verticalSyntheticalDriveBox_directionVector_y_Edit->setText(QString::number(directionVector.y));
			ui.StepOne_verticalSyntheticalDriveBox_directionVector_z_Edit->setText(QString::number(directionVector.z));

			rightProcessingStatus = false;
			//添加到发送信息
			resultMessage.rightCenter.X = "NULL";
			resultMessage.rightCenter.Y = "NULL";
			resultMessage.rightCenter.Z = "NULL";
			resultMessage.directionVector.A = "NULL";
			resultMessage.directionVector.B = "NULL";
			resultMessage.directionVector.C = "NULL";
		}

	}

	if (num == 2010){
		leftCenterResult = centerResult;
		leftCircle_cloudPoints = borderCloud_ptr;
		leftPlaneNormal = planeparam;
		if (planeparamStatus == true)
		{
			if (leftCenterResult.status == true){
				//输出结果
				ui.StepTwo_verticalEngine_normal_x_Edit->setStyleSheet("color:black");
				ui.StepTwo_verticalEngine_normal_y_Edit->setStyleSheet("color:black");
				ui.StepTwo_verticalEngine_normal_z_Edit->setStyleSheet("color:black");

				ui.StepTwo_verticalEngine_center_x_Edit->setStyleSheet("color:black");
				ui.StepTwo_verticalEngine_center_y_Edit->setStyleSheet("color:black");
				ui.StepTwo_verticalEngine_center_z_Edit->setStyleSheet("color:black");

				ui.StepTwo_verticalEngine_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepTwo_verticalEngine_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepTwo_verticalEngine_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepTwo_verticalEngine_center_x_Edit->setText(QString::number(centerResult.center.x));
				ui.StepTwo_verticalEngine_center_y_Edit->setText(QString::number(centerResult.center.y));
				ui.StepTwo_verticalEngine_center_z_Edit->setText(QString::number(centerResult.center.z));

				leftProcessingStatus = true;
				//添加到发送信息
				resultMessage.leftNormal.A = QString::number(planeparam.A);
				resultMessage.leftNormal.B = QString::number(planeparam.B);
				resultMessage.leftNormal.C = QString::number(planeparam.C);
				resultMessage.leftCenter.X = QString::number(centerResult.center.x);
				resultMessage.leftCenter.Y = QString::number(centerResult.center.y);
				resultMessage.leftCenter.Z = QString::number(centerResult.center.z);

			}
			else{
				statusUpdate(QStringLiteral("右侧圆心拟合失败，重新采集！"), 1);
				ui.StepTwo_verticalEngine_normal_x_Edit->setStyleSheet("color:black");
				ui.StepTwo_verticalEngine_normal_y_Edit->setStyleSheet("color:black");
				ui.StepTwo_verticalEngine_normal_z_Edit->setStyleSheet("color:black");

				ui.StepTwo_verticalEngine_center_x_Edit->setStyleSheet("color:red");
				ui.StepTwo_verticalEngine_center_y_Edit->setStyleSheet("color:red");
				ui.StepTwo_verticalEngine_center_z_Edit->setStyleSheet("color:red");

				ui.StepTwo_verticalEngine_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepTwo_verticalEngine_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepTwo_verticalEngine_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepTwo_verticalEngine_center_x_Edit->setText("NULL");
				ui.StepTwo_verticalEngine_center_y_Edit->setText("NULL");
				ui.StepTwo_verticalEngine_center_z_Edit->setText("NULL");

				leftProcessingStatus = true;
				//添加到发送信息
				resultMessage.leftNormal.A = QString::number(planeparam.A);
				resultMessage.leftNormal.B = QString::number(planeparam.B);
				resultMessage.leftNormal.C = QString::number(planeparam.C);
				resultMessage.leftCenter.X = "NULL";
				resultMessage.leftCenter.Y = "NULL";
				resultMessage.leftCenter.Z = "NULL";

			}
		}
		else
		{
			statusUpdate(QStringLiteral("左侧采集出现问题，请调整位置重新采集！"), 1);
			ui.StepTwo_verticalEngine_normal_x_Edit->setStyleSheet("color:red");
			ui.StepTwo_verticalEngine_normal_y_Edit->setStyleSheet("color:red");
			ui.StepTwo_verticalEngine_normal_z_Edit->setStyleSheet("color:red");

			ui.StepTwo_verticalEngine_center_x_Edit->setStyleSheet("color:red");
			ui.StepTwo_verticalEngine_center_y_Edit->setStyleSheet("color:red");
			ui.StepTwo_verticalEngine_center_z_Edit->setStyleSheet("color:red");

			ui.StepTwo_verticalEngine_normal_x_Edit->setText("NULL");
			ui.StepTwo_verticalEngine_normal_y_Edit->setText("NULL");
			ui.StepTwo_verticalEngine_normal_z_Edit->setText("NULL");

			ui.StepTwo_verticalEngine_center_x_Edit->setText("NULL");
			ui.StepTwo_verticalEngine_center_y_Edit->setText("NULL");
			ui.StepTwo_verticalEngine_center_z_Edit->setText("NULL");
			leftProcessingStatus = false;
			//添加到发送信息
			resultMessage.leftNormal.A = "NULL";
			resultMessage.leftNormal.B = "NULL";
			resultMessage.leftNormal.C = "NULL";
			resultMessage.leftCenter.X = "NULL";
			resultMessage.leftCenter.Y = "NULL";
			resultMessage.leftCenter.Z = "NULL";
		}
		

	}
	if (num == 2020){
		rightCenterResult = centerResult;
		rightCircle_cloudPoints = borderCloud_ptr;
		rightPlaneNormal = planeparam;
		if (planeparamStatus == true)
		{
			if (rightCenterResult.status == true){
				//输出结果
				ui.StepTwo_verticalSyntheticalDriveBox_normal_x_Edit->setStyleSheet("color:black");
				ui.StepTwo_verticalSyntheticalDriveBox_normal_y_Edit->setStyleSheet("color:black");
				ui.StepTwo_verticalSyntheticalDriveBox_normal_z_Edit->setStyleSheet("color:black");
				ui.StepTwo_verticalSyntheticalDriveBox_center_x_Edit->setStyleSheet("color:black");
				ui.StepTwo_verticalSyntheticalDriveBox_center_y_Edit->setStyleSheet("color:black");
				ui.StepTwo_verticalSyntheticalDriveBox_center_z_Edit->setStyleSheet("color:black");

				ui.StepTwo_verticalSyntheticalDriveBox_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepTwo_verticalSyntheticalDriveBox_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepTwo_verticalSyntheticalDriveBox_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepTwo_verticalSyntheticalDriveBox_center_x_Edit->setText(QString::number(centerResult.center.x));
				ui.StepTwo_verticalSyntheticalDriveBox_center_y_Edit->setText(QString::number(centerResult.center.y));
				ui.StepTwo_verticalSyntheticalDriveBox_center_z_Edit->setText(QString::number(centerResult.center.z));

				rightProcessingStatus = true;
				//添加到发送信息
				resultMessage.rightNormal.A = QString::number(planeparam.A);
				resultMessage.rightNormal.B = QString::number(planeparam.B);
				resultMessage.rightNormal.C = QString::number(planeparam.C);
				resultMessage.rightCenter.X = QString::number(centerResult.center.x);
				resultMessage.rightCenter.Y = QString::number(centerResult.center.y);
				resultMessage.rightCenter.Z = QString::number(centerResult.center.z);

			}
			else{
				statusUpdate(QStringLiteral("右侧圆心拟合误失败，重新采集！"), 1);
				ui.StepTwo_verticalSyntheticalDriveBox_normal_x_Edit->setStyleSheet("color:black");
				ui.StepTwo_verticalSyntheticalDriveBox_normal_y_Edit->setStyleSheet("color:black");
				ui.StepTwo_verticalSyntheticalDriveBox_normal_z_Edit->setStyleSheet("color:black");
				ui.StepTwo_verticalSyntheticalDriveBox_center_x_Edit->setStyleSheet("color:red");
				ui.StepTwo_verticalSyntheticalDriveBox_center_y_Edit->setStyleSheet("color:red");
				ui.StepTwo_verticalSyntheticalDriveBox_center_z_Edit->setStyleSheet("color:red");
				ui.StepTwo_verticalSyntheticalDriveBox_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepTwo_verticalSyntheticalDriveBox_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepTwo_verticalSyntheticalDriveBox_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepTwo_verticalSyntheticalDriveBox_center_x_Edit->setText("NULL");
				ui.StepTwo_verticalSyntheticalDriveBox_center_y_Edit->setText("NULL");
				ui.StepTwo_verticalSyntheticalDriveBox_center_z_Edit->setText("NULL");

				rightProcessingStatus = true;
				//添加到发送信息
				resultMessage.rightNormal.A = QString::number(planeparam.A);
				resultMessage.rightNormal.B = QString::number(planeparam.B);
				resultMessage.rightNormal.C = QString::number(planeparam.C);
				resultMessage.rightCenter.X = "NULL";
				resultMessage.rightCenter.Y = "NULL";
				resultMessage.rightCenter.Z = "NULL";
			}
		}
		else
		{
			statusUpdate(QStringLiteral("右侧采集出现问题，请调整位置重新采集！"), 1);
			ui.StepTwo_verticalSyntheticalDriveBox_normal_x_Edit->setStyleSheet("color:red");
			ui.StepTwo_verticalSyntheticalDriveBox_normal_y_Edit->setStyleSheet("color:red");
			ui.StepTwo_verticalSyntheticalDriveBox_normal_z_Edit->setStyleSheet("color:red");
			ui.StepTwo_verticalSyntheticalDriveBox_center_x_Edit->setStyleSheet("color:red");
			ui.StepTwo_verticalSyntheticalDriveBox_center_y_Edit->setStyleSheet("color:red");
			ui.StepTwo_verticalSyntheticalDriveBox_center_z_Edit->setStyleSheet("color:red");
			ui.StepTwo_verticalSyntheticalDriveBox_normal_x_Edit->setText("NULL");
			ui.StepTwo_verticalSyntheticalDriveBox_normal_y_Edit->setText("NULL");
			ui.StepTwo_verticalSyntheticalDriveBox_normal_z_Edit->setText("NULL");

			ui.StepTwo_verticalSyntheticalDriveBox_center_x_Edit->setText("NULL");
			ui.StepTwo_verticalSyntheticalDriveBox_center_y_Edit->setText("NULL");
			ui.StepTwo_verticalSyntheticalDriveBox_center_z_Edit->setText("NULL");
			rightProcessingStatus = false;
			//添加到发送信息
			resultMessage.rightNormal.A = "NULL";
			resultMessage.rightNormal.B = "NULL";
			resultMessage.rightNormal.C = "NULL";
			resultMessage.rightCenter.X = "NULL";
			resultMessage.rightCenter.Y = "NULL";
			resultMessage.rightCenter.Z = "NULL";
		}

	}
	if (num == 2030){
		leftFlangeHoleCenterResult = centerResult;
		leftFlangeHoleCircle_cloudPoints = borderCloud_ptr;
		if (rightCenterResult.status == true){
			//输出结果
			ui.StepTwo_verticalEngine_holeCenter_x_Edit->setStyleSheet("color:black");
			ui.StepTwo_verticalEngine_holeCenter_y_Edit->setStyleSheet("color:black");
			ui.StepTwo_verticalEngine_holeCenter_z_Edit->setStyleSheet("color:black");

			ui.StepTwo_verticalEngine_holeCenter_x_Edit->setText(QString::number(centerResult.center.x));
			ui.StepTwo_verticalEngine_holeCenter_y_Edit->setText(QString::number(centerResult.center.y));
			ui.StepTwo_verticalEngine_holeCenter_z_Edit->setText(QString::number(centerResult.center.z));

			leftProcessingStatus = true;
			//添加到发送信息
			resultMessage.leftCenter.X = QString::number(centerResult.center.x);
			resultMessage.leftCenter.Y = QString::number(centerResult.center.y);
			resultMessage.leftCenter.Z = QString::number(centerResult.center.z);

		}
		else{
			statusUpdate(QStringLiteral("右侧圆心拟合失败，重新采集！"), 1);
			ui.StepTwo_verticalEngine_holeCenter_x_Edit->setStyleSheet("color:red");
			ui.StepTwo_verticalEngine_holeCenter_y_Edit->setStyleSheet("color:red");
			ui.StepTwo_verticalEngine_holeCenter_z_Edit->setStyleSheet("color:red");

			ui.StepTwo_verticalEngine_holeCenter_x_Edit->setText("NULL");
			ui.StepTwo_verticalEngine_holeCenter_y_Edit->setText("NULL");
			ui.StepTwo_verticalEngine_holeCenter_z_Edit->setText("NULL");

			leftProcessingStatus = false;
			//添加到发送信息
			resultMessage.leftCenter.X = "NULL";
			resultMessage.leftCenter.Y = "NULL";
			resultMessage.leftCenter.Z = "NULL";
		}

	}
	if (num == 2040){
		rightFlangeHoleCenterResult = centerResult;
		rightFlangeHoleCircle_cloudPoints = borderCloud_ptr;
		if (rightCenterResult.status == true){
			//输出结果
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_x_Edit->setStyleSheet("color:black");
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_y_Edit->setStyleSheet("color:black");
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_z_Edit->setStyleSheet("color:black");

			ui.StepTwo_verticalSyntheticalDriveBox_holeCenter_x_Edit->setText(QString::number(centerResult.center.x));
			ui.StepTwo_verticalSyntheticalDriveBox_holeCenter_y_Edit->setText(QString::number(centerResult.center.y));
			ui.StepTwo_verticalSyntheticalDriveBox_holeCenter_z_Edit->setText(QString::number(centerResult.center.z));

			rightProcessingStatus = true;
			//添加到发送信息
			resultMessage.rightCenter.X = QString::number(centerResult.center.x);
			resultMessage.rightCenter.Y = QString::number(centerResult.center.y);
			resultMessage.rightCenter.Z = QString::number(centerResult.center.z);

		}
		else{
			statusUpdate(QStringLiteral("右侧圆心拟合失败，重新采集！"), 1);
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_x_Edit->setStyleSheet("color:red");
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_y_Edit->setStyleSheet("color:red");
			ui.StepTwo_horizontalFrontDriveBox_holeCenter_z_Edit->setStyleSheet("color:red");
			ui.StepTwo_verticalSyntheticalDriveBox_holeCenter_x_Edit->setText(QString::number(centerResult.center.x));
			ui.StepTwo_verticalSyntheticalDriveBox_holeCenter_y_Edit->setText(QString::number(centerResult.center.y));
			ui.StepTwo_verticalSyntheticalDriveBox_holeCenter_z_Edit->setText(QString::number(centerResult.center.z));

			rightProcessingStatus = false;
			resultMessage.rightCenter.X = "NULL";
			resultMessage.rightCenter.Y = "NULL";
			resultMessage.rightCenter.Z = "NULL";
		}

	}
	if (num == 3010){
		leftCenterResult = centerResult;
		leftCircle_cloudPoints = borderCloud_ptr;
		leftPlaneNormal = planeparam;
		if (planeparamStatus == true)
		{
			if (leftCenterResult.status == true){
				//输出结果
				ui.StepThree_verticalEngine_normal_x_Edit->setStyleSheet("color:black");
				ui.StepThree_verticalEngine_normal_y_Edit->setStyleSheet("color:black");
				ui.StepThree_verticalEngine_normal_z_Edit->setStyleSheet("color:black");
				ui.StepThree_verticalEngine_center_x_Edit->setStyleSheet("color:black");
				ui.StepThree_verticalEngine_center_y_Edit->setStyleSheet("color:black");
				ui.StepThree_verticalEngine_center_z_Edit->setStyleSheet("color:black");

				ui.StepThree_verticalEngine_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepThree_verticalEngine_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepThree_verticalEngine_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepThree_verticalEngine_center_x_Edit->setText(QString::number(centerResult.center.x));
				ui.StepThree_verticalEngine_center_y_Edit->setText(QString::number(centerResult.center.y));
				ui.StepThree_verticalEngine_center_z_Edit->setText(QString::number(centerResult.center.z));

				leftProcessingStatus = true;
				//添加到发送信息
				resultMessage.leftNormal.A = QString::number(planeparam.A);
				resultMessage.leftNormal.B = QString::number(planeparam.B);
				resultMessage.leftNormal.C = QString::number(planeparam.C);
				resultMessage.leftCenter.X = QString::number(centerResult.center.x);
				resultMessage.leftCenter.Y = QString::number(centerResult.center.y);
				resultMessage.leftCenter.Z = QString::number(centerResult.center.z);
			}
			else{
				statusUpdate(QStringLiteral("左侧圆心拟合失败，重新采集！"), 1);
				ui.StepThree_verticalEngine_normal_x_Edit->setStyleSheet("color:black");
				ui.StepThree_verticalEngine_normal_y_Edit->setStyleSheet("color:black");
				ui.StepThree_verticalEngine_normal_z_Edit->setStyleSheet("color:black");
				ui.StepThree_verticalEngine_center_x_Edit->setStyleSheet("color:red");
				ui.StepThree_verticalEngine_center_y_Edit->setStyleSheet("color:red");
				ui.StepThree_verticalEngine_center_z_Edit->setStyleSheet("color:red");

				ui.StepThree_verticalEngine_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepThree_verticalEngine_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepThree_verticalEngine_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepThree_verticalEngine_center_x_Edit->setText("NULL");
				ui.StepThree_verticalEngine_center_y_Edit->setText("NULL");
				ui.StepThree_verticalEngine_center_z_Edit->setText("NULL");

				leftProcessingStatus = true;
				//添加到发送信息
				resultMessage.leftNormal.A = QString::number(planeparam.A);
				resultMessage.leftNormal.B = QString::number(planeparam.B);
				resultMessage.leftNormal.C = QString::number(planeparam.C);
				resultMessage.leftCenter.X = "NULL";
				resultMessage.leftCenter.Y = "NULL";
				resultMessage.leftCenter.Z = "NULL";

			}
		}
		else
		{
			statusUpdate(QStringLiteral("左侧采集出现问题，请调整位置重新采集！"), 1);
			ui.StepThree_verticalEngine_normal_x_Edit->setStyleSheet("color:red");
			ui.StepThree_verticalEngine_normal_y_Edit->setStyleSheet("color:red");
			ui.StepThree_verticalEngine_normal_z_Edit->setStyleSheet("color:red");
			ui.StepThree_verticalEngine_center_x_Edit->setStyleSheet("color:red");
			ui.StepThree_verticalEngine_center_y_Edit->setStyleSheet("color:red");
			ui.StepThree_verticalEngine_center_z_Edit->setStyleSheet("color:red");

			ui.StepThree_verticalEngine_normal_x_Edit->setText("NULL");
			ui.StepThree_verticalEngine_normal_y_Edit->setText("NULL");
			ui.StepThree_verticalEngine_normal_z_Edit->setText("NULL");

			ui.StepThree_verticalEngine_center_x_Edit->setText("NULL");
			ui.StepThree_verticalEngine_center_y_Edit->setText("NULL");
			ui.StepThree_verticalEngine_center_z_Edit->setText("NULL");

			leftProcessingStatus = false;
			//添加到发送信息
			resultMessage.leftNormal.A = "NULL";
			resultMessage.leftNormal.B = "NULL";
			resultMessage.leftNormal.C = "NULL";
			resultMessage.leftCenter.X = "NULL";
			resultMessage.leftCenter.Y = "NULL";
			resultMessage.leftCenter.Z = "NULL";
		}

	}
	if (num == 3020){
		rightCenterResult = centerResult;
		rightCircle_cloudPoints = borderCloud_ptr;
		rightPlaneNormal = planeparam;
		if (planeparamStatus == true)
		{
			if (rightCenterResult.status == true){
				ui.StepThree_verticalSyntheticalDriveBox_normal_x_Edit->setStyleSheet("color:black");
				ui.StepThree_verticalSyntheticalDriveBox_normal_y_Edit->setStyleSheet("color:black");
				ui.StepThree_verticalSyntheticalDriveBox_normal_z_Edit->setStyleSheet("color:black");
				ui.StepThree_verticalSyntheticalDriveBox_center_x_Edit->setStyleSheet("color:black");
				ui.StepThree_verticalSyntheticalDriveBox_center_y_Edit->setStyleSheet("color:black");
				ui.StepThree_verticalSyntheticalDriveBox_center_z_Edit->setStyleSheet("color:black");

				ui.StepThree_verticalSyntheticalDriveBox_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepThree_verticalSyntheticalDriveBox_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepThree_verticalSyntheticalDriveBox_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepThree_verticalSyntheticalDriveBox_center_x_Edit->setText(QString::number(centerResult.center.x));
				ui.StepThree_verticalSyntheticalDriveBox_center_y_Edit->setText(QString::number(centerResult.center.y));
				ui.StepThree_verticalSyntheticalDriveBox_center_z_Edit->setText(QString::number(centerResult.center.z));

				rightProcessingStatus = true;
				//添加到发送信息
				resultMessage.leftNormal.A = QString::number(planeparam.A);
				resultMessage.leftNormal.B = QString::number(planeparam.B);
				resultMessage.leftNormal.C = QString::number(planeparam.C);
				resultMessage.leftCenter.X = QString::number(centerResult.center.x);
				resultMessage.leftCenter.Y = QString::number(centerResult.center.y);
				resultMessage.leftCenter.Z = QString::number(centerResult.center.z);
			}
			else{
				statusUpdate(QStringLiteral("右侧圆心拟合失败，重新采集！"), 1);
				ui.StepThree_verticalSyntheticalDriveBox_normal_x_Edit->setStyleSheet("color:black");
				ui.StepThree_verticalSyntheticalDriveBox_normal_y_Edit->setStyleSheet("color:black");
				ui.StepThree_verticalSyntheticalDriveBox_normal_z_Edit->setStyleSheet("color:black");
				ui.StepThree_verticalSyntheticalDriveBox_center_x_Edit->setStyleSheet("color:red");
				ui.StepThree_verticalSyntheticalDriveBox_center_y_Edit->setStyleSheet("color:red");
				ui.StepThree_verticalSyntheticalDriveBox_center_z_Edit->setStyleSheet("color:red");

				ui.StepThree_verticalSyntheticalDriveBox_normal_x_Edit->setText(QString::number(planeparam.A));
				ui.StepThree_verticalSyntheticalDriveBox_normal_y_Edit->setText(QString::number(planeparam.B));
				ui.StepThree_verticalSyntheticalDriveBox_normal_z_Edit->setText(QString::number(planeparam.C));

				ui.StepThree_verticalSyntheticalDriveBox_center_x_Edit->setText("NULL");
				ui.StepThree_verticalSyntheticalDriveBox_center_y_Edit->setText("NULL");
				ui.StepThree_verticalSyntheticalDriveBox_center_z_Edit->setText("NULL");

				rightProcessingStatus = true;
				//添加到发送信息
				resultMessage.leftNormal.A = QString::number(planeparam.A);
				resultMessage.leftNormal.B = QString::number(planeparam.B);
				resultMessage.leftNormal.C = QString::number(planeparam.C);
				resultMessage.leftCenter.X = "NULL";
				resultMessage.leftCenter.Y = "NULL";
				resultMessage.leftCenter.Z = "NULL";
			}
		}
		else
		{
			statusUpdate(QStringLiteral("右侧采集出现问题，请调整位置重新采集！"), 1);
			ui.StepThree_verticalSyntheticalDriveBox_normal_x_Edit->setText("NULL");
			ui.StepThree_verticalSyntheticalDriveBox_normal_y_Edit->setText("NULL");
			ui.StepThree_verticalSyntheticalDriveBox_normal_z_Edit->setText("NULL");

			ui.StepThree_verticalSyntheticalDriveBox_center_x_Edit->setText("NULL");
			ui.StepThree_verticalSyntheticalDriveBox_center_y_Edit->setText("NULL");
			ui.StepThree_verticalSyntheticalDriveBox_center_z_Edit->setText("NULL");

			rightProcessingStatus = false;
			//添加到发送信息
			resultMessage.leftNormal.A = "NULL";
			resultMessage.leftNormal.B = "NULL";
			resultMessage.leftNormal.C = "NULL";
			resultMessage.leftCenter.X = "NULL";
			resultMessage.leftCenter.Y = "NULL";
			resultMessage.leftCenter.Z = "NULL";

		}

	}
}

void DisplayGUI::receiveStepParam(double CamNoScanSpeed, double CamScanDistance, double CamFirstNoScanDistance, double CamSecondNoScanDistance)
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

void DisplayGUI::resolveSignal(QString info)
{
	if (info.mid(0, 3) == "hor" || info.mid(0, 3) == "ver"){
		vp->engineNumFileName = "E://engineFile/" + info;
		engineNumFileName = "E://engineFile/" + info;
		QDir dir;
		bool ok = dir.mkdir("E://engineFile");
		ok= dir.mkdir(vp->engineNumFileName);
		socketNum = 0;
		//将其保存到TXT文件中
		QString fileName = "ConfigFile/engineNumFileName.txt";//写入文件的目录
		QFile file(fileName);
		file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
		QTextStream out(&file);		
		out << vp->engineNumFileName << "\n";
		file.close();
		//创建文件夹
		createProjectDirectory(info);
		createParamFirle(info);
	}
	else if (info.mid(0, 1) == "H" || info.mid(0, 1)=="V"){
		int infoSize = info.size();
		cout << "infoSize = " << infoSize << endl;
		engineVersion = info.mid(0, 1);
		socketNum = info.mid(1, 3).toInt();
		QString robotPoseStr;
		QString midStr[6];
		int midStrNum = 0;
		robotPoseStr = info.mid(5);
		VisualProcessing::RobotPose robotPose;
		cout << "robotPoseStr = " << robotPoseStr.toStdString() << endl;
		for (int i = 0; i < robotPoseStr.size(); i++)
		{
			if (robotPoseStr.mid(i, 1) != ","){
				midStr[midStrNum].append(robotPoseStr.mid(i, 1));
			}
			else{
				midStrNum++;
			}
		}
		robotPose.X = midStr[0].toDouble();
		robotPose.Y = midStr[1].toDouble();
		robotPose.Z = midStr[2].toDouble();
		robotPose.A = midStr[3].toDouble();
		robotPose.B = midStr[4].toDouble();
		robotPose.C = midStr[5].toDouble();
		cout << "engineVersion:" << engineVersion.toStdString() << endl;
		cout << "socketNum:" << socketNum << endl;
		cout << "robotPose.X:" << robotPose.X << endl;
		cout << "robotPose.Y:" << robotPose.Y << endl;
		cout << "robotPose.Z:" << robotPose.Z << endl;
		cout << "robotPose.A:" << robotPose.A << endl;
		cout << "robotPose.B:" << robotPose.B << endl;
		cout << "robotPose.C:" << robotPose.C << endl;
		emit sendRobotPose(robotPose);
	}
	else{
		socketNum = 999;//错误命令
	}
	

}

void DisplayGUI::saveInformationToJournalFile(QString info)
{
	QDateTime CurrentTime = QDateTime::currentDateTime();
	QString Timestr = CurrentTime.toString("yyyy-MM-dd hh:mm:ss"); // 设置显示的格式
	QString statusStr = Timestr + ": " + info;
	QString fileName = engineNumFileName + "/JournalFile.txt";//写入文件的目录
	QFile file(fileName);
	file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
	QTextStream out(&file);
	out << statusStr << "\n";
	file.close();
}

void DisplayGUI::createProjectDirectory(QString info)
{
	engineNumFileName = "E://engineFile/" + info;
	QDir dir;
	//创建保存图片文件夹
	bool ok = dir.mkdir(engineNumFileName+"/JournalImage");
	//创建输出文件文件夹
	ok = dir.mkdir(engineNumFileName + "/outputFile");
	
	if (info.mid(0, 3) == "hor"){
		//创建工步一文件夹
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne");
		//第一部分
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/firstPart");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/firstPart/frontDriveBoxUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/firstPart/frontDriveBoxCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/firstPart/frontDriveBoxFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/firstPart/frontDriveBoxFinalBorderPoints");
		//第二部分
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/secondPart");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/secondPart/frontDriveBoxUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/secondPart/frontDriveBoxCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/secondPart/frontDriveBoxFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/secondPart/frontDriveBoxFinalBorderPoints");
		//创建工步二文件夹
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo");
		//第一部分
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart/engineUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart/engineCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart/engineFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart/engineFinalBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart/frontDriveBoxUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart/frontDriveBoxCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart/frontDriveBoxFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart/frontDriveBoxFinalBorderPoints");
		//第二部分
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleFinalBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart/frontDriveBoxFlangeHoleUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart/frontDriveBoxFlangeHoleCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart/frontDriveBoxFlangeHoleFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart/frontDriveBoxFlangeHoleFinalBorderPoints");
		//创建工步三文件夹
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree");
		//第一部分
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxFinalBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart/frontDriveBoxUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart/frontDriveBoxCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart/frontDriveBoxFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart/frontDriveBoxFinalBorderPoints");
		//第二部分
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/secondPart");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/secondPart/syntheticalDriveBoxFlangeHoleUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/secondPart/syntheticalDriveBoxFlangeHoleCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/secondPart/syntheticalDriveBoxFlangeHoleFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/secondPart/syntheticalDriveBoxFlangeHoleFinalBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/secondPart/frontDriveBoxFlangeHoleUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/secondPart/frontDriveBoxFlangeHoleCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/secondPart/frontDriveBoxFlangeHoleFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/secondPart/frontDriveBoxFlangeHoleFinalBorderPoints");
		//创建工步四文件夹
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFour");
		//第一部分
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFour/firstPart");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFour/firstPart/syntheticalDriveBoxUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFour/firstPart/syntheticalDriveBoxCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFour/firstPart/syntheticalDriveBoxFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFour/firstPart/syntheticalDriveBoxFinalBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFour/firstPart/frontDriveBoxUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFour/firstPart/frontDriveBoxCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFour/firstPart/frontDriveBoxFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFour/firstPart/frontDriveBoxFinalBorderPoints");
		//第二部分

		//创建工步五文件夹
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFive");
		//第一部分
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFive/firstPart");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFive/firstPart/engineUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFive/firstPart/engineCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFive/firstPart/engineFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFive/firstPart/engineFinalBorderPoints");
		
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFive/firstPart/frontDriveBoxUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFive/firstPart/frontDriveBoxCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFive/firstPart/frontDriveBoxFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepFive/firstPart/frontDriveBoxFinalBorderPoints");
	}
	if (info.mid(0, 3) == "ver"){
		//创建工步一文件夹
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne");
		//第一部分
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/firstPart");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/firstPart/syntheticalDriveBoxUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/firstPart/syntheticalDriveBoxCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/firstPart/syntheticalDriveBoxFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/firstPart/syntheticalDriveBoxFinalBorderPoints");
		//第二部分
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/secondPart");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/secondPart/syntheticalDriveBoxUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/secondPart/syntheticalDriveBoxCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/secondPart/syntheticalDriveBoxFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepOne/secondPart/syntheticalDriveBoxFinalBorderPoints");
		//创建工步二文件夹
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo");
		//第一部分
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart/engineUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart/engineCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart/engineFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart/engineFinalBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart/syntheticalDriveBoxUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart/syntheticalDriveBoxCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart/syntheticalDriveBoxFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/firstPart/syntheticalDriveBoxFinalBorderPoints");
		//第二部分
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleFinalBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart/syntheticalDriveBoxFlangeHoleUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart/syntheticalDriveBoxFlangeHoleCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart/syntheticalDriveBoxFlangeHoleFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepTwo/secondPart/syntheticalDriveBoxFlangeHoleFinalBorderPoints");
		//创建工步三文件夹
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree");
		//第一部分
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxFinalBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart/engineUnifyCirclePoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart/engineCirclePointsThroughDistanceThreshold");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart/engineFilterBorderPoints");
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/firstPart/engineFinalBorderPoints");
		//第二部分
		ok = dir.mkdir(engineNumFileName + "/outputFile/stepThree/secondPart");
	}
	
}

void DisplayGUI::createParamFirle(QString info)
{
	engineNumFileName = "E://engineFile/" + info;

	QString fileName;
	ofstream outfile;
	if (info.mid(0, 3) == "hor")
	{
		//横置发动机
		//工位1
		//光栅尺反馈导轨移动量表头
		fileName = engineNumFileName + "/outputFile/stepOne/GratingData.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "方位" << "," << "第一次采集时离原点距离" << "," << "第二次采集时离原点距离" << "\n";
		outfile.close();
		//前传动箱止口圆心的两次测量的方向向量表头
		fileName = engineNumFileName + "/outputFile/stepOne/frontDriveBoxDirectionVector.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "方向矢量X" << "," << "方向矢量Y" << "," << "方向矢量Z" << "\n";
		outfile.close();
		//工位1第一部分
		fileName = engineNumFileName + "/outputFile/stepOne/firstPart/engineParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "法矢A" << "," << "法矢B" << "," << "法矢C" << "\n";
		outfile.close();

		fileName = engineNumFileName + "/outputFile/stepOne/firstPart/frontDriveBoxParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" << "\n";
		outfile.close();
		//工位1第二部分
		fileName = engineNumFileName + "/outputFile/stepOne/secondPart/frontDriveBoxParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" << "\n";
		outfile.close();
		//工位2
		fileName = engineNumFileName + "/outputFile/stepTwo/GratingData.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "方位" << "," << "第一次采集时离原点距离" << "," << "第二次采集时离原点距离" << "\n";
		outfile.close();
		//工位2第一部分
		fileName = engineNumFileName + "/outputFile/stepTwo/firstPart/engineParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();

		fileName = engineNumFileName + "/outputFile/stepTwo/firstPart/frontDriveBoxParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();
		//工位2第二部分
		fileName = engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();

		fileName = engineNumFileName + "/outputFile/stepTwo/secondPart/frontDriveBoxFlangeHoleParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();
		//工位3
		fileName = engineNumFileName + "/outputFile/stepThree/GratingData.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "方位" << "," << "第一次采集时离原点距离" << "," << "第二次采集时离原点距离" << "\n";
		outfile.close();
		//工位3第一部分
		fileName = engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();

		fileName = engineNumFileName + "/outputFile/stepThree/firstPart/frontDriveBoxParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();
		//工位3第二部分
		fileName = engineNumFileName + "/outputFile/stepThree/secondPart/syntheticalDriveBoxFlangeHoleParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();

		fileName = engineNumFileName + "/outputFile/stepThree/secondPart/frontDriveBoxFlangeHoleParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();
		//工位4
		fileName = engineNumFileName + "/outputFile/stepFour/GratingData.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "方位" << "," << "第一次采集时离原点距离" << "," << "第二次采集时离原点距离" << "\n";
		outfile.close();
		//工位4第一部分
		fileName = engineNumFileName + "/outputFile/stepFour/firstPart/syntheticalDriveBoxParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();

		fileName = engineNumFileName + "/outputFile/stepFour/firstPart/frontDriveBoxParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();
		//工位4第二部分

		//工位5
		fileName = engineNumFileName + "/outputFile/stepFive/GratingData.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "方位" << "," << "第一次采集时离原点距离" << "," << "第二次采集时离原点距离" << "\n";
		outfile.close();
		//工位5第一部分
		fileName = engineNumFileName + "/outputFile/stepFive/firstPart/engineParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();

		fileName = engineNumFileName + "/outputFile/stepFive/firstPart/frontDriveBoxParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();
		//工位5第二部分
	}
	
	if (info.mid(0, 3) == "ver")
	{
		//纵置发动机
		//工位1
		//光栅尺反馈导轨移动量表头
		fileName = engineNumFileName + "/outputFile/stepOne/GratingData.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "方位" << "," << "第一次采集时离原点距离" << "," << "第二次采集时离原点距离" << "\n";
		outfile.close();
		//综合传动箱止口圆心的两次测量的方向向量表头
		fileName = engineNumFileName + "/outputFile/stepOne/syntheticalDriveBoxDirectionVector.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "方向矢量X" << "," << "方向矢量Y" << "," << "方向矢量Z" << "\n";
		outfile.close();
		//工位1第一部分
		fileName = engineNumFileName + "/outputFile/stepOne/firstPart/engineParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "法矢A" << "," << "法矢B" << "," << "法矢C" << "\n";
		outfile.close();

		fileName = engineNumFileName + "/outputFile/stepOne/firstPart/syntheticalDriveBoxParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" << "\n";
		outfile.close();
		//工位1第二部分
		fileName = engineNumFileName + "/outputFile/stepOne/secondPart/syntheticalDriveBoxParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" << "\n";
		outfile.close();
		//工位2
		fileName = engineNumFileName + "/outputFile/stepTwo/GratingData.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "方位" << "," << "第一次采集时离原点距离" << "," << "第二次采集时离原点距离" << "\n";
		outfile.close();
		//工位2第一部分
		fileName = engineNumFileName + "/outputFile/stepTwo/firstPart/engineParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();

		fileName = engineNumFileName + "/outputFile/stepTwo/firstPart/syntheticalDriveBoxParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();
		//工位2第二部分
		fileName = engineNumFileName + "/outputFile/stepTwo/secondPart/engineFlangeHoleParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();

		fileName = engineNumFileName + "/outputFile/stepTwo/secondPart/syntheticalDriveBoxFlangeHoleParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();
		//工位3
		fileName = engineNumFileName + "/outputFile/stepThree/GratingData.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "方位" << "," << "第一次采集时离原点距离" << "," << "第二次采集时离原点距离" << "\n";
		outfile.close();
		//工位3第一部分
		fileName = engineNumFileName + "/outputFile/stepThree/firstPart/engineParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();

		fileName = engineNumFileName + "/outputFile/stepThree/firstPart/syntheticalDriveBoxParam.csv";
		outfile.open(fileName.toStdString());
		outfile << "时间" << "," << "工位" << "," << "圆心X" << "," << "圆心Y" << "," << "圆心Z" << "," << "圆心拟合状态" << ","
			<< "法矢A" << "," << "法矢B" << "," << "法矢C" << "," << "边缘点到圆心的平均距离" "\n";
		outfile.close();
		//工位3第二部分
	}
	
}

/**
*横置发动机信号为101 102等  
*纵置发动机信号在横置发动机信号基础上在末尾加上0，例1010 1020
*/

void DisplayGUI::receiveImageNum()
{
	int num = ui.numEdit->text().toInt();
	sendImageNum(num);
}

void DisplayGUI::receiveImageAndSave(HObject image, int num, QString name)
{
	//显示图片
	displayImage(image, num);
	//保存图片
	SaveImage(image, name);
}

void DisplayGUI::endThread()
{
	proThread->quit();
	proThread->wait();

}

















