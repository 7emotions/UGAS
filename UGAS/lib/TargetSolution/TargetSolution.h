#pragma once
/*
Creation Date: 2022/10/19
Latest Update: 2022/10/19
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ����װ�װ���Ϣ��ʱ��ά���Ͻ�������
  ����Ƿ������еļ�¼Ϊͬһ��װ�װ�/Ŀ��
  ά����ʶ��Ŀ�꼯���������
- ͨ�����÷��ؽ��
*/
#include "Common/UniversalStruct.h"
#include "GimbalSerial/GimbalSerial.h"
#include "Common/DebugTools/DebugHeader.h"
#include "Target.h"

class TargetSolution {
protected:
	serial::GimbalSerial& _com;
	std::vector<Target> _targets;

	virtual cv::Vec3f SolvePNP(const ArmorPlate& armor) = 0;
public:
	TargetSolution(serial::GimbalSerial& com) :_com(com) {}

	const std::vector<Target>& GetResultRefer()
		const { return _targets; }
	virtual void Solve(const std::vector<ArmorPlate>& armors) = 0;
};
