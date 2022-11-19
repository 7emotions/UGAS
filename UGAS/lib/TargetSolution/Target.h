#pragma once
/*
Creation Date: 2022/10/19
Latest Update: 2022/10/19
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- �ṩ�洢����Ŀ����Ϣ�ķ�װ
*/
#include <opencv2/opencv.hpp>
#include "Common/DebugTools/DebugHeader.h"

class TargetSolution; // ��Ԫ������Ԥ����

class Target {
private:
	// �����ں�TimeStamp��Target(�����ٸ�)
	cv::Vec3f _position, _speed;
public:
	Target(cv::Vec3f position = cv::Vec3f(), cv::Vec3f speed = cv::Vec3f()) :
		_position(position), _speed(speed) {}

	cv::Vec3f Predict(int milliSec) const {
		return _position + _speed * milliSec;
	}

	friend class TargetSolution;
};
