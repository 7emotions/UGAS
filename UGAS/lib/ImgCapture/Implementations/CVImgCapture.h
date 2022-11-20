#pragma once
/*
Creation Date: 2022/10/12
Latest Update: 2022/10/12
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ��װOpenCV�Դ�VideoCapture��ʹ�����ImgCapture�Ľӿ�
- ��������Ƶ����
*/
#include "../ImgCapture.h"

class CVImgCapture : public ImgCapture, private cv::VideoCapture {
private:
	TimeStamp _startTime;
public:
	virtual void init(void* fileName);
	virtual void read(Img& img);
};
