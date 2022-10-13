#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/11
Developer(s): 21 THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- �ṩһ����ʱ������ʵ�֣��ͽ� ImgPretreat Ver 1.0
*/
#include "../ImgPretreat.h"

class ImgPretreat_V1 : public ImgPretreat {
public:
	ImgPretreat_V1(serial::GimbalSerial& com) :
		ImgPretreat(com) {}

	void GetPretreated(Img& img);
};
