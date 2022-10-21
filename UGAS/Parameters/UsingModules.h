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
#ifdef _WIN32
#include "../lib/GimbalSerial/Implementations/Windows/WindowsGimbalSerial.h"
#define GIMBAL_SERIAL	serial::WindowsGimbalSerial
#else
#include "../lib/GimbalSerial/Implementations/Linux/LinuxGimbalSerial.h"
#define GIMBAL_SERIAL	serial::LinuxGimbalSerial
#endif
#include "../lib/ImgCapture/Implementations/CVImgCapture.h"
#include "../lib/ImgPretreat/Implementations/ImgPretreat_V1.h"
#include "../lib/ArmorFinder/Implementations/ArmorIdentifier_V1.h"
#include "../lib/ArmorFinder/NumberIdentifier/Implementations/NullNumberIdentifier.h"
#include "../lib/TargetSolution/Implementations/TargetSolution_V1.h"
#include "../lib/TrackingStrategy/Implementations/TrackingStrategy_V1.h"
#include "../lib/Trajectory/Implementations/Trajectory_FEM.h"

#define IMG_CAPTURE		CVImgCapture
#define IMG_PRETREAT	ImgPretreat_V1
#define ARMOR_IDENTIFY	ArmorIdentifier_V1
#define NUMBER_IDENTIFY	NullNumberIdentifier
#define TARGET_SOLUTION TargetSolution_V1
#define TRACK_STRATEGY  TrackingStrategy_V1
#define TRAJECTORY		Trajectory_FEM
