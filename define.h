#pragma once

#ifndef DEFINE_H_
#define DEFINE_H_

#define MMTOCOUNT 1000;
#define DEGTOCOUNT 12000;

//���崫�������Ϣ�ṹ��
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
//��λö��
enum stepName{
	h_stepOneFirstPart=101/*��ⷢ����ֹ�ڷ�ʸ*/,
	h_stepOneSecondPart = 102/*���ǰ������ֹ����Բ��1*/,
	h_stepOneThirdPart = 103/*���ǰ������ֹ����Բ��2��Բ��1��2���*/,

	h_stepTwoFirstPart = 201/*��ⷢ����ֹ��ƽ�淨ʸ��Բ������*/,
	h_stepTwoSecondPart = 202/*���ǰ������ֹ��ƽ�淨ʸ��Բ������*/,
	h_stepTwoThirdPart = 203/*��ⷢ����ֹ��һ��������Բ������*/,
	h_stepTwoForthPart = 204/*���ǰ������ֹ��һ��������Բ������*/,

	h_stepThreeFirstPart = 301/*����ۺϴ�����ֹ��ƽ�淨ʸ��Բ������*/,
	h_stepThreeSecondPart = 302/*���ǰ������ֹ��ƽ�淨ʸ��Բ������*/,
	h_stepThreeThirdPart = 303/*����ۺϴ�����ֹ��һ��������Բ������*/,
	h_stepThreeForthPart = 304/*���ǰ������ֹ��һ��������Բ������*/,

	h_stepFourFirstPart = 401/*����۴����仨���׵�ƽ�淨ʸ��Բ��*/,
	h_stepFourSecondPart = 402/*���ǰ�����仨�����ƽ�淨ʸ��Բ��*/,
	h_stepFourThirdPart = 403/*����۴����仨����λ��*/,
	h_stepFourForthPart = 404/*���ǰ�����仨����λ��*/,

	h_stepFiveFirstPart = 501/*��ⷢ���������׵�ƽ�淨ʸ��Բ��*/,
	h_stepFiveSecondPart = 502/*���ǰ�����仨�����ƽ�淨ʸ��Բ��*/,
	h_stepFiveThirdPart = 503/*��ⷢ����������λ��*/,
	h_stepFiveForthPart = 504/*���ǰ�����仨����λ��*/,
	
	
	
	//�ָ�
	v_stepOneFirstPart=1010,
	v_stepOneSecondPart=1020,
	v_stepTwoFirstPart=2010,
	v_stepTwoSecondPart=2020,
	v_stepThreeFirstPart = 3010, 
	v_stepThreeSecondPart = 3020,
};
//��λ���Ҳɼ�ö��
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
	
	//�ָ�
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