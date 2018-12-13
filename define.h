#pragma once

#ifndef DEFINE_H_
#define DEFINE_H_

#define MMTOCOUNT 1000;
#define DEGTOCOUNT 12000;

//定义传输测量信息结构体
struct NormalMessage
{
	QString A;
	QString B;
	QString C;
	void clear(){
		this->A = "NULL";
		this->B = "NULL";
		this->C = "NULL";
	}
};
struct CenterMessage
{
	QString X;
	QString Y;
	QString Z;
	void clear(){
		this->X = "NULL";
		this->Y = "NULL";
		this->Z = "NULL";
	}
};
struct DVectorMessage
{
	QString A;
	QString B;
	QString C;
	void clear(){
		this->A = "NULL";
		this->B = "NULL";
		this->C = "NULL";
	}
};
struct ResultMessage
{
	NormalMessage leftNormal;
	CenterMessage leftCenter;
	QString leftPhaseAngle;
	NormalMessage rightNormal;
	CenterMessage rightCenter;
	QString rightPhaseAngle;
	DVectorMessage directionVector;

	void clear(){
		leftNormal.clear();
		leftCenter.clear();
		leftPhaseAngle = "NULL";
		rightNormal.clear();
		rightCenter.clear();
		rightPhaseAngle = "NULL";
		directionVector.clear();
	}
};
//工位枚举
enum stepName{
	h_stepOneFirstPart=101/*检测发动机止口法矢*/,
	h_stepOneSecondPart = 102/*检测前传动箱止口面圆心1*/,
	h_stepOneThirdPart = 103/*检测前传动箱止口面圆心2，圆心1，2相减*/,

	h_stepTwoFirstPart = 201/*检测发动机止口平面法矢和圆心坐标*/,
	h_stepTwoSecondPart = 202/*检测前传动箱止口平面法矢和圆心坐标*/,
	h_stepTwoThirdPart = 203/*检测发动机止口一个法兰孔圆心坐标*/,
	h_stepTwoForthPart = 204/*检测前传动箱止口一个法兰孔圆心坐标*/,

	h_stepThreeFirstPart = 301/*检测综合传送箱止口平面法矢和圆心坐标*/,
	h_stepThreeSecondPart = 302/*检测前传动箱止口平面法矢和圆心坐标*/,
	h_stepThreeThirdPart = 303/*检测综合传送箱止口一个法兰孔圆心坐标*/,
	h_stepThreeForthPart = 304/*检测前传动箱止口一个法兰孔圆心坐标*/,

	h_stepFourFirstPart = 401/*检测综传动箱花键孔的平面法矢和圆心*/,
	h_stepFourSecondPart = 402/*检测前传动箱花键轴的平面法矢和圆心*/,
	h_stepFourThirdPart = 403/*检测综传动箱花键相位角*/,
	h_stepFourForthPart = 404/*检测前传动箱花键相位角*/,

	h_stepFiveFirstPart = 501/*检测发动机花键孔的平面法矢和圆心*/,
	h_stepFiveSecondPart = 502/*检测前传动箱花键轴的平面法矢和圆心*/,
	h_stepFiveThirdPart = 503/*检测发动机花键相位角*/,
	h_stepFiveForthPart = 504/*检测前传动箱花键相位角*/,
	
	
	
	//分割
	v_stepOneFirstPart=1010,
	v_stepOneSecondPart=1020,
	v_stepTwoFirstPart=2010,
	v_stepTwoSecondPart=2020,
	v_stepThreeFirstPart = 3010, 
	v_stepThreeSecondPart = 3020,
};
//工位左右采集枚举
enum stepCapture{
	left_hOneFirstP = 10101,
	right_hOneFirstP = 102,
	//left_hOneSecondP = 10201,
	right_hOneSecondP = 103,
	//left_hOneThirdP = 10301,
	//right_hOneThirdP = 10302,

	left_hTwoFirstP = 201,
	right_hTwoFirstP = 202,
	left_hTwoSecondP = 203,
	right_hTwoSecondP = 204,
	/*left_hTwoThirdP = 20301,
	right_hTwoThirdP = 20302,
	left_hTwoForthP = 20401,
	right_hTwoForthP = 20402,*/

	left_hThreeFirstP = 301,
	right_hThreeFirstP = 302,
	left_hThreeSecondP = 303,
	right_hThreeSecondP = 304,
	/*left_hThreeThirdP = 30301,
	right_hThreeThirdP = 30302,
	left_hThreeForthP = 30401,
	right_hThreeForthP = 30402,*/

	left_hFourFirstP = 401,
	right_hFourFirstP = 402,
	left_hFourSecondP = 403,
	right_hFourSecondP = 404,
	/*left_hFourThirdP = 40301,
	right_hFourThirdP = 40302,
	left_hFourForthP = 40401,
	right_hFourForthP = 40402,*/

	left_hFiveFirstP = 501,
	right_hFiveFirstP = 502,
	left_hFiveSecondP = 503,
	right_hFiveSecondP = 504,
	/*left_hFiveThirdP = 40301,
	right_hFiveThirdP = 40302,
	left_hFiveForthP = 40401,
	right_hFiveForthP = 40402,*/
	
	//分割
	left_vOneFP = 1010,
	right_vOneFP = 1030,
	left_vOneSP = 1020,
	right_vOneSP = 1040,

	left_vTwoFP = 2010,
	right_vTwoFP = 2030,
	left_vTwoSP = 2020,
	right_vTwoSP = 2040,

	left_vThreeFP = 3010,
	right_vThreeFP = 3030,
	left_vThreeSP = 3020,
	right_vThreeSP = 3040,

};

#endif