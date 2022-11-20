#pragma once
/*
Creation Date: 2022/11/20
Latest Update: 2022/11/20
Developer(s): 22-QZH
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ��װLinuxHTCameraCapture��WindowsHTCamearCapture
- �õ���ƽ̨��HTCameraCapture
*/

#ifdef _WIN32

#include "Windows/WindowsHTCameraCapture.h"
class HTCameraCapture: public WindowsHTCameraCapture {
public:
	HTCameraCapture() : WindowsHTCameraCapture() { }
};

#else

#include "Linux/LinuxHTCameraCapture.h"
class HTCameraCapture : public LinuxHTCameraCapture {
public:
	HTCameraCapture() : LinuxHTCameraCapture() { }
};

#endif

