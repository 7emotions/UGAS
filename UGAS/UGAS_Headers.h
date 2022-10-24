#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- �������� UGAS ֱ����Ҫ��ͷ�ļ�
  �����ⲿ���ļ����Զ�����ļ�
*/

// External Library Files
#include <opencv2/opencv.hpp>

// Custom Library Files
#include "Parameters/DebugSettings.h"
#include "Parameters/Parameters.h"
#include "Parameters/UsingModules.h"
#include "lib/GimbalSerial/GimbalSerial.h"
#include "lib/ImgCapture/ImgCapture.h"
#include "lib/ImgPretreat/ImgPretreat.h"
#include "lib/ArmorFinder/ArmorIdentifier.h"
#include "lib/TargetSolution/TargetSolution.h"
#include "lib/TrackingStrategy/TrackingStrategy.h"
#include "lib/Trajectory/Trajectory.h"

#include "lib/Common/FPSCounter/FPSCounter.h"
