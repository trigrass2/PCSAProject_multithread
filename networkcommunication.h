#ifndef NETWORKCOMMUNICATION_H
#define NETWORKCOMMUNICATION_H

#include <QWidget>
#include "ui_networkcommunication.h"
#include <QTcpSocket>
#include <iostream>
#include<QMessageBox>
#include<QFile>
#include "waitforfeedbackistruction.h"

class NetworkCommunication : public QWidget
{
	Q_OBJECT

public:
	NetworkCommunication(QWidget *parent = 0);
	~NetworkCommunication();
	void send(QString information);
	void ReadIPconfig();
	QTcpSocket *mySocket;
	bool socketConnectionStatus;
	int variable[5];
	QString infoRecently;//储存最近的一次命令
signals:
	void stringReceived(QString);
	void sendInfo(QString,int);
public slots:
	void on_ConnectButton_clicked();
	void on_DisconnectButton_clicked();
	void on_SendButton_clicked();
	void on_SaveButton_clicked();
	void displayReceive(QString info);
	void checkConnection();//检测通讯是否断开
	void erro();//通讯错误
	void receiveSendAgain();//接收重新发送命令
private slots:
	void readyRead();
private:
	Ui::NetworkCommunication ui;
	WaitForFeedbackIstruction *wait;
};

#endif // NETWORKCOMMUNICATION_H
