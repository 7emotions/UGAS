#pragma once
/*
Creation Date: 2022/10/19
Latest Update: 2022/10/19
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ���嵯���������ͳһ�ӿ�
*/
#include "GimbalSerial/GimbalSerial.h"
#include "TargetSolution/Target.h"
#include "Common/DebugTools/DebugHeader.h"

class Trajectory {
protected:
	serial::GimbalSerial& _com;
public:
	Trajectory(serial::GimbalSerial& com) :_com(com) {}

	virtual void GetShotAngle(const Target& target, TimeStamp ImgTime, double& yaw, double& pitch) = 0;
};
