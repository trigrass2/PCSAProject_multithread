#ifndef SERIALCOMMUNICATION_H
#define SERIALCOMMUNICATION_H

#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QString>
#include <QByteArray>
#include<windows.h>
#include<iostream>
#include<vector>
class SerialCommunication : public QObject
{
	Q_OBJECT

public:
	//SerialCommunication(QObject *parent);
	SerialCommunication();
	~SerialCommunication();
	void clearGratingData();
	void readGratingData(std::vector<double> &runDis);

public slots:
	void sendPlC(QByteArray byte);

private:
	//´®¿ÚÍ¨Ñ¶
	QSerialPort *Qserial;


};
#endif // SERIALCOMMUNICATION_H