#pragma once
/*
Creation Date: 2022/10/12
Latest Update: 2022/10/12
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- �����ȡͼ�����ͨ�ýӿ�
Class public functions:
- init(void*)
	��ʼ��ͼ������
- read(Img&)
	��ȡһ֡ͼ��ͨ���޸����õķ�ʽ���ؽ��
*/
#include <opencv2/opencv.hpp>
#include "../../lib/Common/UniversalStruct.h"
#include "../Common/DebugTools/DebugHeader.h"

class ImgCapture {
public:
	virtual void init(void*) = 0;
	virtual void read(Img&) = 0;
};
