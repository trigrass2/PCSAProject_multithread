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
};
struct CenterMessage
{
	QString X;
	QString Y;
	QString Z;
};
struct DVectorMessage
{
	QString A;
	QString B;
	QString C;
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
};
#endif