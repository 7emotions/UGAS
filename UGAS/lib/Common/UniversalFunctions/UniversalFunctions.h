#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ������ʵ������ͨ�ü��㸨������
*/
#include <opencv.hpp>
#include "../UniversalStruct.h"

double P2PDis(const cv::Point2f& a, const cv::Point2f& b);
double P2PDis(const cv::Point3f& a, const cv::Point3f& b);

double malposition(const LightBar& LBl, const LightBar& LBr);

double CoordinateDistance(double a, double b);
