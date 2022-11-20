#pragma once
/*
Creation Date: 2022/11/19
Latest Update: 2022/11/19
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ͨ��FEM(����Ԫģ��)�ķ�ʽģ�ⵯ��������Ŀ��Ƕ�
*/
#include "../Trajectory.h"

class Trajectory_FEM : public Trajectory {
private:
	cv::Point3f _3Dposition;
	cv::Point2f _2Dposition;

	// ������angle�Ƕȵ���distanceʱ��z��߶�
	double Analyze(double distance, double angle, double altitudeTarget, double& flyTime);
	// �����������ʱ��Ԥ��Ŀ���˶���
	void Iterate(cv::Point3f position, double& pitch, double& flyTime);
public:
	void GetShotAngle(const int targetID, TimeStamp ImgTime, double& yaw, double& pitch);
};
