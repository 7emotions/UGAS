#pragma once
/*
Creation Date: 2023/03/17
Latest Update: 2023/03/17
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- װ�װ�PnP����Ľӿ���
- ��ͼ���������Ϊ�����˱�������
*/


#include <optional>

#include <opencv2/opencv.hpp>

#include "Control/Gimbal/Gimbal.h"
#include "Core/Identifier/Armor/ArmorStruct.h"


/*
  position: �����˱�������
  ����ϵԭ��������˱������ĵ��غ�
  ����ϵz����ֱ���ϣ�xy��������������ʱ����ָ̨���йأ���̨��ת��Ӱ��xyz��
*/
struct ArmorPlate3d {
	ArmorID id;
	GimbalAttitude gimbalAttitude;
	cv::Point3f position;
	ArmorPlate3d(ArmorID _id, const GimbalAttitude& _gimbalAttitude, const cv::Point3f& _position)
		: id(_id), gimbalAttitude(_gimbalAttitude), position(_position) { }
};


class ArmorPnPSolverInterface {
public:
	virtual ~ArmorPnPSolverInterface() = default;

	virtual void UpdateGimbalAttitude(double pitch, double yaw) = 0;
	// ͼ���������Ϊ�����˱�������
	virtual std::optional<ArmorPlate3d> Solve(const ArmorPlate& armor, bool isLargeArmor) = 0;
};