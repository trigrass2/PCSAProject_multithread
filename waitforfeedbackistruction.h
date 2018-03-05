#pragma once

#include <QThread>
#include<iostream>
using  std::cout;
using std::endl;
class WaitForFeedbackIstruction : public QThread
{
	Q_OBJECT

public:
	WaitForFeedbackIstruction(QObject *parent);
	~WaitForFeedbackIstruction();

	void run();
	bool waitStatus;
signals:
	void sendAgain();
};
