#include "motioncontrol.h"
using  std::cout;
using  std::endl;
MotionControl::MotionControl(QWidget *parent)
: QWidget(parent)
{
	ui.setupUi(this);

	setMFCmode();
	disAllErrMsgBox(1);
		
}

MotionControl::~MotionControl()
{
	deleteDCE(pComD2);
	//deleteDCE(pComDD);
	
}

void MotionControl::setLeadRailVelocity(double speed)
{
	cout << "设置导轨运动速度为：" << speed << endl;
	double speed_count = speed * MMTOCOUNT;
	SetVarN("X_vel_max", speed_count, 0, pComD2);
}

void MotionControl::setLeadRailMotionDistance(double distance)
{
	
	double X_ref_pos;
	GetVarN("X_ref_pos", &X_ref_pos, 0, pComD2); //从'X_ref_pos'变量中读取转台当前位置的count值
	double X_run = X_ref_pos + distance*MMTOCOUNT;
	SetVarN("X_trg", X_run, 0, pComD2);
}

void MotionControl::platformLtoRRotate()
{
	cout << "转台转动" << endl;

	double X_ref_pos;
	GetVarN("X_ref_pos", &X_ref_pos, 0, pComDD); //从'X_ref_pos'变量中读取转台当前位置的count值
	//SetVarN("X_vel_max", 120000, 0, pComDD);
	SetVarN("X_trg", X_ref_pos - 2160000, 0, pComDD);
	int Run = -1;
	Sleep(100);
	getStateN(0, "X_run", &Run, pComDD);

	while (Run == 1)
	{
		getStateN(0, "X_run", &Run, pComDD);
	}

}

void MotionControl::platformRtoLRotate()
{
	cout << "转台转动" << endl;

	double X_ref_pos;
	GetVarN("X_ref_pos", &X_ref_pos, 0, pComDD); //从'X_ref_pos'变量中读取转台当前位置的count值
	//SetVarN("X_vel_max", 120000, 0, pComDD);
	SetVarN("X_trg", X_ref_pos + 2160000, 0, pComDD);
	int Run = -1;
	Sleep(100);
	getStateN(0, "X_run", &Run, pComDD);

	while (Run == 1)
	{
		getStateN(0, "X_run", &Run, pComDD);
	}
}


void MotionControl::reset(double startPosition, double endPosition)
{
	double X_distance;
	double X_ref_pos;
	double speed=35;
	speed = speed * MMTOCOUNT;
	SetVarN("X_vel_max", speed, 0, pComD2);
	Sleep(200);
	//获取快速移动距离
	/*X_distance = endPosition - startPosition;
	cout << "导轨快速移动距离为：" << X_distance << endl;
	GetVarN("X_ref_pos", &X_ref_pos, 0, pComD2);
	SetVarN("X_vel_max", speed, 0, pComD2);
	SetVarN("X_trg", X_ref_pos - (X_distance - 20000), 0, pComD2);
	CheckRun();*/
	//导轨缓慢回原点
	QString str = "05 30 30 46 46 42 54 30 30 31 58 30 30 31 35 31";
	QByteArray Bstr = str.toLatin1();
	QByteArray Byte = QByteArray::fromHex(Bstr);
	emit toPLCInfo(Byte);
	//cout << "发送数据PLC" << endl;
	//Sleep(100);
	////检测导轨停止转动
	//CheckRun();
	////转台回原点
	//SetVarN("X_vel_max", 72000, 0, pComDD);//set speed 6deg/s
	//SetVarN("in_xor", 0000000000010001, 0, pComDD);
	//int Run = 2;
	//Sleep(500);
	//getStateN(0, "X_run", &Run, pComDD);
	//while (Run == 1)
	//{
	//	getStateN(0, "X_run", &Run, pComDD);
	//}
	//SetVarN("in_xor", 0000000000010000, 0, pComDD);
	//Sleep(800);
	//double X_ref_pos;
	//GetVarN("X_ref_pos", &X_ref_pos, 0, pComDD);
	//SetVarN("X_trg", X_ref_pos + 1627200, 0, pComDD);
	//cout << "转台复位完成" << endl;
	//Sleep(100);
	//sendInfo(QStringLiteral("运动机构复位"), 0);
	//cout << "复位" << endl;
}

void MotionControl::on_forwardButton_pressed()
{
	double X_ref_pos;
	QString speedStr;
	double speed;
	speedStr = ui.speedEdit->text();
	speed = speedStr.toDouble();
	speed = speed * MMTOCOUNT;
	GetVarN("X_ref_pos", &X_ref_pos, 0, pComD2); //从'X_ref_pos'变量中读取转台当前位置的count值
	SetVarN("X_vel_max", speed, 0, pComD2);
	SetVarN("X_trg", X_ref_pos + 9000000, 0, pComD2);
}

void MotionControl::on_forwardButton_released()
{
	double X_ref_pos;
	QString speedStr;
	double speed;
	speedStr = ui.speedEdit->text();
	speed = speedStr.toDouble();
	speed = speed * MMTOCOUNT;
	GetVarN("X_ref_pos", &X_ref_pos, 0, pComD2); //从'X_ref_pos'变量中读取转台当前位置的count值
	SetVarN("X_vel_max", speed, 0, pComD2);
	SetVarN("X_trg", X_ref_pos + 1000, 0, pComD2);
}

void MotionControl::on_backwardButton_pressed()
{
	double X_ref_pos;
	QString speedStr;
	double speed;
	speedStr = ui.speedEdit->text();
	speed = speedStr.toDouble();
	speed = speed * MMTOCOUNT;
	GetVarN("X_ref_pos", &X_ref_pos, 0, pComD2); //从'X_ref_pos'变量中读取转台当前位置的count值
	SetVarN("X_vel_max", speed, 0, pComD2);
	SetVarN("X_trg", X_ref_pos - 9000000, 0, pComD2);
}

void MotionControl::on_backwardButton_released()
{
	double X_ref_pos;
	QString speedStr;
	double speed;
	speedStr = ui.speedEdit->text();
	speed = speedStr.toDouble();
	speed = speed * MMTOCOUNT;
	GetVarN("X_ref_pos", &X_ref_pos, 0, pComD2); //从'X_ref_pos'变量中读取转台当前位置的count值
	SetVarN("X_vel_max", speed, 0, pComD2);
	SetVarN("X_trg", X_ref_pos - 1000, 0, pComD2);
}


void MotionControl::initializationMotion()
{
	bConnectedD2 = -1;
	bConnectedDD = -1;
	//连接导轨
	pComD2 = openDCE("C:\\HIWIN\\dce\\", "D2.dce");//打开DD马达
	bConnectedD2 = setComPar(8, 8, 1, 0, 0, 0, 0, 0, 50, 200, 6, pComD2);
	SetVarN("X_acc", 200000, 0, pComD2);//set acceleration
	SetVarN("X_dcc", 200000, 0, pComD2);//set deceleration
	SetVarN("X_vel_max", 40000, 0, pComD2);//set speed
	//连接转台
	pComDD = openDCE("C:\\HIWIN\\dce\\", "DD.dce");//连接DD驱动器
	bConnectedDD = setComPar(2, 8, 1, 0, 0, 0, 0, 0, 50, 200, 6, pComDD);
	SetVarN("X_acc", 2400000, 0, pComDD);//set acceleration
	SetVarN("X_dcc", 2400000, 0, pComDD);//set deceleration
	SetVarN("X_vel_max", 180000, 0, pComDD);//set speed
	//cout << "bConnectedD2 = " << bConnectedD2 << endl;
	if (bConnectedD2 == 0 && bConnectedDD==0){
		sendInfo(QStringLiteral("运动机构连接成功"),0);
	}


	else{
		sendInfo(QStringLiteral("运动机构连接失败"),1);
	}
}

void MotionControl::on_leftButton_pressed()
{
	
	double X_ref_pos=0;
	QString speedStr;
	double speed;
	speedStr = ui.revolutionEdit->text();
	speed = speedStr.toDouble();
	speed = speed * DEGTOCOUNT;
	SetVarN("X_vel_max", speed, 0, pComDD);
	GetVarN("X_ref_pos", &X_ref_pos, 0, pComDD); //从'X_ref_pos'变量中读取转台当前位置的count值
	SetVarN("X_trg", X_ref_pos + 8000000, 0, pComDD);
}

void MotionControl::on_leftButton_released()
{
	double X_ref_pos;
		
	GetVarN("X_ref_pos", &X_ref_pos, 0, pComDD); //从'X_ref_pos'变量中读取转台当前位置的count值
	
	SetVarN("X_trg", X_ref_pos + 6000, 0, pComDD);
}

void MotionControl::on_rightButton_pressed()
{
	double X_ref_pos;
	QString speedStr;
	double speed;
	speedStr = ui.revolutionEdit->text();
	speed = speedStr.toDouble();
	speed = speed * DEGTOCOUNT;
	SetVarN("X_vel_max", speed, 0, pComDD);
	GetVarN("X_ref_pos", &X_ref_pos, 0, pComDD); //从'X_ref_pos'变量中读取转台当前位置的count值
	
	SetVarN("X_trg", X_ref_pos -8000000, 0, pComDD);
}

void MotionControl::on_rightButton_released()
{
	double X_ref_pos;
	GetVarN("X_ref_pos", &X_ref_pos, 0, pComDD); //从'X_ref_pos'变量中读取转台当前位置的count值
	
	SetVarN("X_trg", X_ref_pos -6000, 0, pComDD);
}

void MotionControl::on_resetButton_clicked()
{
	
	if (ui.railBox->isChecked()){
		cout << "railButton" << endl;
		//导轨回原点
		QString str = "05 30 30 46 46 42 54 30 30 31 58 30 30 31 32 31";
		QByteArray Bstr = str.toLatin1();
		QByteArray Byte = QByteArray::fromHex(Bstr);
		emit toPLCInfo(Byte);
		Sleep(800);
		//检测导轨停止转动
		CheckRun();
	}
	if (ui.rotateBox->isChecked()){
		cout << "rotateButton" << endl;
		//转台回原点
		SetVarN("X_vel_max", 72000, 0, pComDD);//set speed 6deg/s
		SetVarN("in_xor", 0000000000010001, 0, pComDD);
		int Run = 2;
		Sleep(500);
		getStateN(0, "X_run", &Run, pComDD);
		while (Run == 1)
		{
			getStateN(0, "X_run", &Run, pComDD);
		}
		SetVarN("in_xor", 0000000000010000, 0, pComDD);
		Sleep(800);
		double X_ref_pos;
		GetVarN("X_ref_pos", &X_ref_pos, 0, pComDD);
		SetVarN("X_trg", X_ref_pos - 1612800, 0, pComDD);
		cout << "转台复位完成" << endl;
		Sleep(100);
	}
}

void MotionControl::leadRailMotion(double distance)
{
	double X_ref_pos;
	double distanceCount;
	double speed = 10;
	speed = speed * MMTOCOUNT;
	distanceCount = distance * MMTOCOUNT;
	cout << "distanceCount = " << distanceCount << endl;
	GetVarN("X_ref_pos", &X_ref_pos, 0, pComD2); //从'X_ref_pos'变量中读取转台当前位置的count值
	SetVarN("X_vel_max", speed, 0, pComD2);
	SetVarN("X_trg", X_ref_pos + distanceCount, 0, pComD2);
	int Run = -1;
	Sleep(100);
	getStateN(0, "X_run", &Run, pComD2);
	//检测导轨是否在运动
	while (Run==1)
	{
		getStateN(0, "X_run", &Run, pComD2);
	}
}

void MotionControl::CheckRun()
{
	int RunD2 = -1;
	
	Sleep(500);
	getStateN(0, "X_run", &RunD2, pComD2);
	while (RunD2 == 1)
	{
		getStateN(0, "X_run", &RunD2, pComD2);
		//Sleep(500);
	}
}

double MotionControl::GetCurrentPositon()
{
	double X_ref_pos;
	GetVarN("X_ref_pos", &X_ref_pos, 0, pComD2); //从'X_ref_pos'变量中读取转台当前位置的count值
	cout <<endl<< "当前位置为：" << X_ref_pos << " count, " << "公称尺寸为：" << X_ref_pos / 1000 << " mm" << endl << endl;
	return X_ref_pos;
}

void MotionControl::on_rotateButton_clicked()
{
	double X_ref_pos;
	if (ui.rightBox->isChecked() && ui.leftBox->isChecked()){
		QMessageBox box;
		box.setWindowTitle(tr("Error"));
		box.setIcon(QMessageBox::Information);
		box.setText(QStringLiteral("请不要多重选择"));
		box.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
		box.exec();
	}
	else{
		if (ui.leftBox->isChecked()){
			GetVarN("X_ref_pos", &X_ref_pos, 0, pComDD);
			SetVarN("X_trg", X_ref_pos + 2160000, 0, pComDD);
		}
		if (ui.rightBox->isChecked()){
			GetVarN("X_ref_pos", &X_ref_pos, 0, pComDD);
			SetVarN("X_trg", X_ref_pos - 2160000, 0, pComDD);
		}
	}
	
}





