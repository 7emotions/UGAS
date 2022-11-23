#pragma once
/*
Creation Date: 2022/11/23
Latest Update: 2022/11/23
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ��װOpenCV�Դ�VideoCapture��ʹ�����ImgCapture�Ľӿ�
- ��������Ƶ���룬���ո���ת��һ֡
*/
#include "../ImgCapture.h"

class CVVideoFrameCapture : public ImgCapture, private cv::VideoCapture {
private:
	cv::Mat _lastFrame;
public:
	virtual void init(void* fileName);
	virtual void read(Img& img);
};
