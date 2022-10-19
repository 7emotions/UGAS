#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ���ð�������ʹ�õ�ģ���ʵ��
- ����ʹ�����͵ĺ�
*/
#include "../lib/GimbalSerial/Implementations/Windows/WindowsGimbalSerial.h"
#include "../lib/ImgCapture/Implementations/CVImgCapture.h"
#include "../lib/ImgPretreat/Implementations/ImgPretreat_V1.h"
#include "../lib/ArmorFinder/Implementations/ArmorIdentifier_V1.h"
#include "../lib/ArmorFinder/NumberIdentifier/Implementations/NullNumberIdentifier.h"
#include "../lib/TargetSolution/Implementations/TargetSolution_V1.h"

#define GIMBAL_SERIAL	serial::WindowsGimbalSerial
#define IMG_CAPTURE		CVImgCapture
#define IMG_PRETREAT	ImgPretreat_V1
#define ARMOR_IDENTIFY	ArmorIdentifier_V1
#define NUMBER_IDENTIFY	NullNumberIdentifier
#define TARGET_SOLUTION TargetSolution_V1
