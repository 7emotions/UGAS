#pragma once
/*
Creation Date: 2022/11/23
Latest Update: 2022/11/23
Developer(s): 22-Qzh 22-Ljc
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ����ʶ��V1�汾
- ʹ��OTSU��CNN
*/
#include "../NumberIdentifier.h"
#include <opencv2\opencv.hpp>

class NumberIdentifier_V1 : public NumberIdentifier {
private:
	cv::dnn::Net _net;
public:
	void init(void* model);
	short Identify(const Img& imgGray, const ArmorPlate& region);
};
