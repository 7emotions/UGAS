#pragma once

/*
Creation Date: 2022/11/20
Latest Update: 2022/11/20
Developer(s): 22-QZH
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- �ṩһ�����ĳ�����ʵ��
*/
#include "../../ImgCapture.h"

class LinuxHTCameraCapture : public ImgCapture
{
protected:
	LinuxHTCameraCapture();

public:
	void init(void*) {}
	void read(Img&) {}
};

