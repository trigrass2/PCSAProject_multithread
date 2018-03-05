#ifndef MOTIONCONTROL_H
#define MOTIONCONTROL_H
#include "stdafx.h"
#include <QWidget>
#include "ui_motioncontrol.h"
#include <iostream>
#include "mpint.h"
#include <stdio.h>
#include"define.h"
#include <QMessageBox>

class MotionControl : public QWidget
{
	Q_OBJECT

public:
	MotionControl(QWidget *parent = 0);
	~MotionControl();
	//导轨及转台初始化
	void initializationMotion();
	//设置导轨速度
	void setLeadRailVelocity(double speed);
	//设置导轨运动距离
	void setLeadRailMotionDistance(double distance);
	//导轨不采集情况下运动
	void leadRailMotion(double distance);
	//转台从左到右转动180度
	void platformLtoRRotate();
	//转台从右到左转动180度
	void platformRtoLRotate();
	//导轨与转台复位
	void reset(double startPosition,double endPosition);
	//检测导轨和转台是否运动
	void CheckRun();
	//获取当前位置
	double GetCurrentPositon();
public slots:
	//导轨运动
	void on_forwardButton_pressed();
	void on_forwardButton_released();
	void on_backwardButton_pressed();
	void on_backwardButton_released();
	//转台运动
	void on_leftButton_pressed();
	void on_leftButton_released();
	void on_rightButton_pressed();
	void on_rightButton_released();
	//复位
	void on_resetButton_clicked();
	//转台左右位置旋转
	void on_rotateButton_clicked();
private:
	Ui::MotionControl ui;
	//声明导轨
	MDCE* pComD2 = NULL;
	//声明转台
	MDCE* pComDD = NULL;
	//声明导轨
	int bConnectedD2;
	//声明转台
	int bConnectedDD;
signals:
	void sendInfo(QString,int);
	void toPLCInfo(QByteArray Byte);
};

#endif // MOTIONCONTROL_H
