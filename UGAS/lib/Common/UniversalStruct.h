#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ��������������ͨ�ýṹ��
*/
#include <opencv2/opencv.hpp>
#include "DebugTools/DebugHeader.h"

enum Team { Red = 1, Blue = 2 };

typedef unsigned long long TimeStamp;

class MatWithTimeStamp :public cv::Mat {
public:
	TimeStamp timeStamp;

	MatWithTimeStamp() :timeStamp(0) {}
	MatWithTimeStamp(const Mat& img) :Mat(img), timeStamp(0) {}
};
typedef MatWithTimeStamp Img;

struct LightBar {
	cv::Point2f top, bottom;
	float angle;

	// ����Զ�����Angle��ʵ��Ҫ��ʶ��ĵط���ʾ��ʽͬ��
	// ����ʲôʱ����ͳһ�ı�׼������ô���Ȳ�ʵ���������
	LightBar(cv::Point2f _top, cv::Point2f _bottom);
	LightBar(cv::Point2f _top, cv::Point2f _bottom, float angle) :
		top(_top), bottom(_bottom), angle(angle) {}
};

class ArmorPlate {
public:
	std::vector<cv::Point2f> points;
	short id;

	ArmorPlate() :id(0) {} // ����֧������
	ArmorPlate(const LightBar& left, const LightBar& right, short _id = 0):
		id(_id) { Set(left, right, _id); }

	// U���ͣ���PNP������ʾ˳��ͬ�����ϴ��봫�е�˳�򣩣�
	void Set(const LightBar& left, const LightBar& right, short _id = 0) {
		points.push_back(left.top); points.push_back(left.bottom);
		points.push_back(right.bottom); points.push_back(right.top);
		if (_id) id = _id;
	}
	cv::Point2f center() const {
		if (points.size() != 4)
			throw_with_trace(std::runtime_error, "Invalid ArmorPlate object");
		return (points[0] + points[1] + points[2] + points[3]) / 4;
	}
};
