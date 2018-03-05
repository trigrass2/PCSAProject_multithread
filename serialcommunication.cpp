#include "serialcommunication.h"
using  std::cout;
using  std::endl;
SerialCommunication::SerialCommunication()
	
{

	//连接PLC
	Qserial = new QSerialPort;
	Qserial->setPortName("COM10");
	Qserial->open(QIODevice::ReadWrite);
	Qserial->setBaudRate(9600);
	Qserial->setDataBits(QSerialPort::Data7);
	Qserial->setParity(QSerialPort::OddParity);
	Qserial->setStopBits(QSerialPort::OneStop);
	Qserial->setFlowControl(QSerialPort::NoFlowControl);
}

SerialCommunication::~SerialCommunication()
{
	delete Qserial;
}

void SerialCommunication::sendPlC(QByteArray byte)
{
	Qserial->write(byte);
	Qserial->waitForBytesWritten(3);
	Qserial->flush();
}

void SerialCommunication::clearGratingData()
{
	QString str = "05 30 30 46 46 42 54 30 30 31 58 30 30 31 36 31";
	QByteArray Bstr = str.toLatin1();
	QByteArray Byte = QByteArray::fromHex(Bstr);
	int a = Qserial->write(Byte);
	Qserial->flush();
	Qserial->waitForBytesWritten(3);
	cout << "clear Grating Data" << endl;
	Qserial->waitForReadyRead(200);
	QByteArray val_1 = Qserial->readAll();
}

void SerialCommunication::readGratingData(std::vector<double> &runDis)
{
	QString str = "05 30 30 46 46 42 54 30 30 31 58 30 30 31 34 31";
	QByteArray Bstr = str.toLatin1();
	QByteArray Byte = QByteArray::fromHex(Bstr);
	int a = Qserial->write(Byte);
	Qserial->flush();
	Qserial->waitForBytesWritten(3);

	Qserial->waitForReadyRead(200);
	QByteArray val_1 = Qserial->readAll();


	QString str_1 = "05 30 30 46 46 57 52 30 44 30 30 31 36 30 31";
	QByteArray arr_1 = str_1.toLatin1();
	QByteArray arr_2 = QByteArray::fromHex(arr_1);
	Qserial->write(arr_2);
	Qserial->waitForBytesWritten(3);
	Qserial->flush();
	Qserial->waitForReadyRead(200);
	val_1 = Qserial->readAll();
	QString str_2 = val_1;
	//ui.textEdit->append("D16=" + str_2);//输出低位D16的数据内容


	QString str_3 = "05 30 30 46 46 57 52 30 44 30 30 31 37 30 31";
	QByteArray arr_3 = str_3.toLatin1();
	QByteArray arr_4 = QByteArray::fromHex(arr_3);
	Qserial->write(arr_4);
	Qserial->waitForBytesWritten(100);
	Qserial->flush();
	Qserial->waitForReadyRead(200);
	QByteArray val_2 = Qserial->readAll();
	QString str_4 = val_2;
	//ui.textEdit->append("D17=" + str_4);//输出高位D17的数据内容


	//////////////////////////分隔线///////////////////////////
	if (val_2[5] == '0')

	{
		QByteArray val_11 = val_1.mid(5, 4);
		QByteArray val_22 = val_2.mid(5.4);
		QByteArray val_all = val_22.append(val_11);
		QByteArray val_Hex;

		for (int i = 0; i < 9; i++)
		{
			if (val_all[i] == '\x3')//val_all[i] == '0' ||
			{
				continue;
			}
			val_Hex.push_back(val_all[i]);

		}

		QString str_Hex = val_Hex.data();
		//ui.textEdit->append(str_Hex);//输出十六进制读数
		//输出十进制的读数
		bool ok;
		int num_1 = str_Hex.toInt(&ok, 16);
		double num_2 = (double)num_1 / 1000;
		runDis.push_back(num_2);
		//QString str_value = QString::number(num_2);
		//ui.textEdit->append("distance=" + str_value + "mm");
	}
	else if (val_2[5] == 'F')
	{
		QByteArray val_11 = val_1.mid(5, 4);
		QByteArray val_22 = val_2.mid(5.4);
		QByteArray val_all = val_22.append(val_11);
		QByteArray val_Hex;

		for (int i = 0; i < 9; i++)
		{
			if (val_all[i] == '\x3')//val_all[i] == '0' ||
			{
				continue;
			}
			val_Hex.push_back(val_all[i]);
		}

		QString str_Hex = val_Hex.data();
		//ui.textEdit->append(str_Hex);//输出十六进制读数

		bool ok;
		uint num_1 = str_Hex.toUInt(&ok, 16);
		int num_11 = -1 - (~num_1);
		double num_2 = (double)num_11 / 1000;
		runDis.push_back(num_2);
		//QString str_value = QString::number(num_2);
		//ui.textEdit->append("distance=" + str_value + "mm");
	}
	else
	{
		//ui.textEdit->setTextColor("red");
		//ui.textEdit->append("There is an error, please retest the count !");
		//ui.textEdit->setTextColor("black");
		runDis.push_back(-1000000);
	}

}

