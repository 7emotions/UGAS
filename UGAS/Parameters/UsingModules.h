#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/11
Developer(s): 21 THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ���ð�������ʹ�õ�ģ���ʵ��
- ����ʹ�����͵ĺ�
*/
#include "../lib/GimbalSerial/Implementations/Windows/WindowsGimbalSerial.h"
#include "../lib/ImgCapture/Implementations/CVVideoCapture.h"
#include "../lib/ImgPretreat/Implementations/ImgPretreat_V1.h"

#define GIMBAL_SERIAL	serial::WindowsGimbalSerial
#define IMG_CAPTURE		CVVideoCapture
#define IMG_PRETREAT	ImgPretreat_V1
