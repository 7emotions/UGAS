#pragma once
/*
Creation Date: 2022/11/16
Latest Update: 2022/11/20
Developer(s): 22-QZH
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ��װHT��ҵ�����SDK(windows)
- ����������ͷ����
*/
#include "../../ImgCapture.h"

#ifdef _WIN64
#pragma comment(lib, "MVCAMSDK_X64.lib")
#else
#pragma comment(lib, "MVCAMSDK.lib")
#endif
#include "../../../../third-party/HTCameraSDK/Windows/CameraApi.h"


class WindowsHTCameraCapture :public ImgCapture {
private:
	bool initialized = false;

	HANDLE _hDispThread;         //ͼ��ץȡ�̵߳ľ��
	CameraHandle _hCamera;       //��������������ͬʱʹ��ʱ���������������	
	tSdkFrameHead _sFrameInfo;   //���ڱ��浱ǰͼ��֡��֡ͷ��Ϣ
	BYTE* _pFrameBuffer;         //���ڽ�ԭʼͼ������ת��ΪRGB�Ļ�����
	 
protected:
	WindowsHTCameraCapture() = default;
	~WindowsHTCameraCapture();

public:
	//always let useLess = nullptr
	virtual void init(void* useLess);
	virtual void read(Img& img);
};