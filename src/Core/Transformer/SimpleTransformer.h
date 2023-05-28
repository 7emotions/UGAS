#pragma once
/*
Creation Date: 2023/05/26
Latest Update: 2023/05/26
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
�����������ϵת��������ʹ�������ǣ������������Ϊ�������ˮƽ��Ŀ��װ�װ����ģ��������Ϊ������ġ���̨���ĺ͵��跢����غϡ�
*/

#include <Eigen/Dense> 


class SimpleTransformer {
public:
	/*! �����������ϵ�µ�װ�װ���ά���꣬ͨ���������Ϊ�������ˮƽ��Ŀ��װ�װ����ģ��������̨��ǰ��pitch��Ƕȣ���������ת����
	* ע�⣺��ͬ��IMU�õ���Transformerÿֻ֡��һ��ʵ����һ�����ÿһ��ʶ�𵽵�װ�װ壬��Ӧ��ʹ�����Ӧʵ������SimpleTransformer��������ת����
	* \param target �������ϵ�µ�װ�װ���ά����
	*/
	explicit SimpleTransformer(const Eigen::Vector3d& target) {
		pitch = -atan2(target.z(), target.x());
	}

	Eigen::Vector3d CameraLink2GimbalLink(const Eigen::Vector3d& srcPos) const {
		return srcPos;
	}

	Eigen::Vector3d Link2Gyro(const Eigen::Vector3d& srcPos) const {
		return {
			srcPos.x() * cos(pitch) - srcPos.z() * sin(pitch),
			srcPos.y(),
			srcPos.z() * cos(pitch) + srcPos.x() * sin(pitch)
		};
	}

	Eigen::Vector3d Gyro2Link(const Eigen::Vector3d& srcPos) const {
		return {
			srcPos.x() * cos(pitch) + srcPos.z() * sin(pitch),
			srcPos.y(),
			srcPos.z() * cos(pitch) - srcPos.x() * sin(pitch)
		};
	}

	Eigen::Vector3d GimbalGyro2MuzzleGyro(const Eigen::Vector3d& srcPos) const {
		return srcPos;
	}

private:
	// ������ļ�����̨������̬��������ѭ���ֶ���
	double pitch;
};