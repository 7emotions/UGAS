#pragma once
/*
Creation Date: 2022/10/12
Latest Update: 2022/10/12
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ��װOpenCV�Դ�VideoCapture��ʹ�����ImgCapture�Ľӿ�
- ����������ͷ����
*/
#include "../ImgCapture.h"

class CVVideoCapture :public ImgCapture, private cv::VideoCapture {
public:
	virtual void init(void* camIndex);
	virtual void read(Img& img);
};
