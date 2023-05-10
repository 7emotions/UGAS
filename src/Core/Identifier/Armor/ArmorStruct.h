#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 22-QZH
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- װ�װ�͵��������ݽṹ
*/

#include <opencv2/opencv.hpp>

#include "Util/Debug/Log.h"

struct LightBar {
	cv::Point2f top, bottom;
	float angle;

	// ����Զ�����Angle��ʵ��Ҫ��ʶ��ĵط���ʾ��ʽͬ��
	// ����ʲôʱ����ͳһ�ı�׼������ô���Ȳ�ʵ���������
	// ToDo:������
	// LightBar(cv::Point2f _top, cv::Point2f _bottom);
	LightBar(cv::Point2f _top, cv::Point2f _bottom, float angle) :
		top(_top), bottom(_bottom), angle(angle) {}
};


enum class ArmorID : uint8_t {
	Unknown = 0,             // �޷�ʶ��
	Hero = 1,                // Ӣ��
	Engineer = 2,            // ����
	InfantryIII = 3,         // ���Ų���
	InfantryIV = 4,          // �ĺŲ���
	InfantryV = 5,           // ��Ų���
	Sentinel = 6,            // �ڱ�
};

struct ArmorPlate {
	ArmorPlate(const LightBar& left, const LightBar& right, ArmorID armorId = ArmorID::Unknown, bool isLargeArmor = false) : id(armorId), is_large_armor(isLargeArmor) {
		// �ϴ��봫�е�˳��U���ͣ���PNP������ʾ˳��ͬ����
		// ��opencvʹ�õ�˳��ͬ������Ҫ�ĵ��������ط���װ�װ����������ʶ��PNP����)
		points.push_back(left.top); points.push_back(left.bottom);
		points.push_back(right.bottom); points.push_back(right.top);
	}

	// U���ͣ���PNP������ʾ˳��ͬ�����ϴ��봫�е�˳�򣩣�
	/*void Set(const LightBar& left, const LightBar& right, short _id = 0) {
		points.push_back(left.top); points.push_back(left.bottom);
		points.push_back(right.bottom); points.push_back(right.top);
		if (_id) id = _id;
	}*/
	
	cv::Point2f center() const {
		if (points.size() != 4)
			throw_with_trace(std::runtime_error, "Invalid ArmorPlate object");
		return (points[0] + points[1] + points[2] + points[3]) / 4;
	}

	/*std::vector<cv::Point2f> OffsetPoints(const cv::Point2f offset) const {
		std::vector<cv::Point2f> offsetPoints;
		for_each(points.begin(), points.end(), [&](const cv::Point2f& point) {
			offsetPoints.push_back(point + offset);
		});
		return offsetPoints;
	}*/

	std::vector<cv::Point2f> points;
	ArmorID id;
	bool is_large_armor;
};