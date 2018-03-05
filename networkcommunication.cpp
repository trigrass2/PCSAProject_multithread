#include "networkcommunication.h"
using  std::cout;
using  std::endl;

NetworkCommunication::NetworkCommunication(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	mySocket = new QTcpSocket;
	wait = new WaitForFeedbackIstruction(0);
	ReadIPconfig();
	connect(mySocket, SIGNAL(readyRead()), this, SLOT(readyRead()));
	connect(this, &NetworkCommunication::stringReceived, this, &NetworkCommunication::displayReceive);
	connect(mySocket, &QTcpSocket::disconnected, this, &NetworkCommunication::checkConnection);//如果断开连接
	connect(mySocket, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(erro()));//显示错误

	connect(wait, &WaitForFeedbackIstruction::sendAgain, this, &NetworkCommunication::receiveSendAgain);//连接多线程，接收重新发送命令
	socketConnectionStatus = false;
	
}

NetworkCommunication::~NetworkCommunication()
{
	
	delete mySocket;
	delete wait;
	mySocket = nullptr;
	wait = nullptr;
}

void NetworkCommunication::send(QString information)
{
	//cout << "Sent: " << information << endl;
	mySocket->write(information.toStdString().c_str());
	mySocket->waitForBytesWritten(3);
	mySocket->flush();
	infoRecently = information;
	if (information != "OK")
	{
		wait->waitStatus = true;
	}
	wait->start();
}

void NetworkCommunication::readyRead()
{ 
	cout << "Read: ";
	QString info = mySocket->readAll();
	cout << info.toStdString() << endl;
	if (info == "OK"){
		wait->waitStatus = false;
	}
	else{
		emit stringReceived(info);
	}
	
}

void NetworkCommunication::on_ConnectButton_clicked()
{
	cout << "Connecting..." << endl;
	QString hostIPAddress = ui.lineEdit_1->text() + "." + ui.lineEdit_2->text() + "." + ui.lineEdit_3->text() + "." + ui.lineEdit_4->text();
	QString hostPort = ui.lineEdit_5->text();
	//QString hostIPAddress = QString::number(variable[0]) + "." + QString::number(variable[1]) + "." + QString::number(variable[2]) + "." + QString::number(variable[3]) ;
	//QString hostPort = QString::number(variable[4]);
	cout << hostIPAddress.toStdString() << endl;
	cout << "Port: " << hostPort.toStdString() << endl;;
	
	mySocket->connectToHost(hostIPAddress, (qint16)hostPort.toInt());
	if (mySocket->waitForConnected(3000)){
		cout << "Connected!" << endl;
		ui.lineEdit_1->setEnabled(false);
		ui.lineEdit_2->setEnabled(false);
		ui.lineEdit_3->setEnabled(false);
		ui.lineEdit_4->setEnabled(false);
		ui.lineEdit_5->setEnabled(false);

		socketConnectionStatus = true;
		sendInfo(QStringLiteral("网络通讯连接成功"),0);
	}
	else{
		cout << "not Connected!" << endl;
		sendInfo(QStringLiteral("网络通讯连接失败"),1);
		socketConnectionStatus = false;
	}
}

void NetworkCommunication::on_DisconnectButton_clicked()
{
	cout << "TCP disconnected" << endl;
	ui.lineEdit_1->setEnabled(true);
	ui.lineEdit_2->setEnabled(true);
	ui.lineEdit_3->setEnabled(true);
	ui.lineEdit_4->setEnabled(true);
	ui.lineEdit_5->setEnabled(true);
	sendInfo(QStringLiteral("网络通讯连接断开"),1);
	mySocket->close();
	socketConnectionStatus = false;
	
}

void NetworkCommunication::on_SendButton_clicked()
{
	mySocket->write(ui.sendEdit->toPlainText().toStdString().c_str());
}

void NetworkCommunication::displayReceive(QString info)
{
	ui.ReceiveEdit->setText(info);
}

void NetworkCommunication::checkConnection()
{
	cout << "网络通讯连接断开" << endl;
	cout << "重新连接网络通讯..." << endl;
	sendInfo(QStringLiteral("网络通讯连接断开"),1);
	sendInfo(QStringLiteral("重新连接网络通讯..."),0);
	on_ConnectButton_clicked();

}

void NetworkCommunication::erro()
{
	cout << "AN erro: ";
	QString str = "AN erro: ";
	str=str.append(mySocket->errorString());//此处为错误打印
	cout << str.toStdString();
	emit sendInfo(str,1);
}

void NetworkCommunication::on_SaveButton_clicked()
{
	QMessageBox box;
	box.setWindowTitle(tr("Information"));
	box.setIcon(QMessageBox::Information);
	box.setText(tr("Do you save the datas?"));
	box.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
	if (box.exec() == QMessageBox::Yes)
	{

		//写入文件
		QString fileName = "ConfigFile/IPconfig.txt";//写入文件的目录
		QFile file(fileName);
		if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
		{
			QMessageBox::warning(this, "ConfigInSteps", "can't open", QMessageBox::Yes);
		}
		QTextStream out(&file);		
		out << ui.lineEdit_1->text().toInt() << "\n";
		out << ui.lineEdit_2->text().toInt() << "\n";
		out << ui.lineEdit_3->text().toInt() << "\n";
		out << ui.lineEdit_4->text().toInt() << "\n";
		out << ui.lineEdit_5->text().toInt() << "\n";
		file.close();
		//关闭使能
	}

}

void NetworkCommunication::ReadIPconfig()
{
	QString fileName = "ConfigFile/IPconfig.txt";
	QFile file(fileName);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		QMessageBox::warning(this, "Warnning", "File can't open", QMessageBox::Yes);
	}
	QTextStream in(&file);
	
	for (int i = 0; i < 5; i++){
		variable[i] = in.readLine().toInt();
	}
	ui.lineEdit_1->setText(QString::number(variable[0], 10));
	ui.lineEdit_2->setText(QString::number(variable[1], 10));
	ui.lineEdit_3->setText(QString::number(variable[2], 10));
	ui.lineEdit_4->setText(QString::number(variable[3], 10));
	ui.lineEdit_5->setText(QString::number(variable[4], 10));
}

void NetworkCommunication::receiveSendAgain()
{
	send(infoRecently);
}

