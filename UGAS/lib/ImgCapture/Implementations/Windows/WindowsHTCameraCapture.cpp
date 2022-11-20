#include "WindowsHTCameraCapture.h"


WindowsHTCameraCapture::~WindowsHTCameraCapture() {
	CameraUnInit(_hCamera);
	CameraAlignFree(_pFrameBuffer);
}


void WindowsHTCameraCapture::init(void* useLess) {
	if (initialized) {
		throw_with_trace(std::runtime_error, "Already initialized");
	}
	else {
		tSdkCameraDevInfo sCameraList[10];
		INT iCameraNums;
		CameraSdkStatus status;
		tSdkCameraCapbility sCameraInfo;

		//ö���豸������豸�б�
		iCameraNums = 10;//����CameraEnumerateDeviceǰ��������iCameraNums = 10����ʾ���ֻ��ȡ10���豸�������Ҫö�ٸ�����豸�������sCameraList����Ĵ�С��iCameraNums��ֵ

		if (CameraEnumerateDevice(sCameraList, &iCameraNums) != CAMERA_STATUS_SUCCESS || iCameraNums == 0)
		{
			throw_with_trace(std::runtime_error, "No camera was found!");
		}

		//��ʾ���У�����ֻ����������һ���������ˣ�ֻ��ʼ����һ�������(-1,-1)��ʾ�����ϴ��˳�ǰ����Ĳ���������ǵ�һ��ʹ�ø�����������Ĭ�ϲ���.
		//In this demo ,we just init the first camera.
		if ((status = CameraInit(&sCameraList[0], -1, -1, &_hCamera)) != CAMERA_STATUS_SUCCESS)
		{
			LOG(ERROR) << "Error code is " << status << CameraGetErrorString(status);
			throw_with_trace(std::runtime_error, "Failed to init the camera");
		}


		//Get properties description for this camera.
		CameraGetCapability(_hCamera, &sCameraInfo);//"��ø��������������"

		_pFrameBuffer = (BYTE*)CameraAlignMalloc(sCameraInfo.sResolutionRange.iWidthMax * sCameraInfo.sResolutionRange.iWidthMax * 3, 16);

		if (sCameraInfo.sIspCapacity.bMonoSensor)
		{
			CameraSetIspOutFormat(_hCamera, CAMERA_MEDIA_TYPE_MONO8);
		}

		//strcpy_s(g_CameraName, sCameraList[0].acFriendlyName);

		CameraPlay(_hCamera);

		initialized = true;
	}
}

void WindowsHTCameraCapture::read(Img& img) {
	if (initialized) {
		BYTE* pbyBuffer;
		CameraSdkStatus status;
		if (status = CameraGetImageBuffer(_hCamera, &_sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
		{
			//����õ�ԭʼ����ת����RGB��ʽ�����ݣ�ͬʱ����ISPģ�飬��ͼ����н��룬������������ɫУ���ȴ���
			//�ҹ�˾�󲿷��ͺŵ������ԭʼ���ݶ���Bayer��ʽ��
			if (status = CameraImageProcess(_hCamera, pbyBuffer, _pFrameBuffer, &_sFrameInfo) == CAMERA_STATUS_SUCCESS) {
				//����SDK��װ�õ���ʾ�ӿ�����ʾͼ��,��Ҳ���Խ�m_pFrameBuffer�е�RGB����ͨ��������ʽ��ʾ������directX,OpengGL,�ȷ�ʽ��
				CameraImageOverlay(_hCamera, _pFrameBuffer, &_sFrameInfo);

				// ����SDK���������Ĭ���Ǵӵ׵����ģ�ת��ΪOpencvͼƬ��Ҫ��һ�´�ֱ����
				CameraFlipFrameBuffer(_pFrameBuffer, &_sFrameInfo, 1);

				cv::Mat matImage(
					cv::Size(_sFrameInfo.iWidth, _sFrameInfo.iHeight),
					_sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
					_pFrameBuffer
				);
				img = matImage;
			}
			else {
				LOG(ERROR) << "CameraImageProcess status = " << status;
				throw_with_trace(std::runtime_error, "Failed to execute CameraImageProcess");
			}

			//�ڳɹ�����CameraGetImageBuffer�󣬱������CameraReleaseImageBuffer���ͷŻ�õ�buffer��
			//�����ٴε���CameraGetImageBufferʱ�����򽫱�����ֱ�������߳��е���CameraReleaseImageBuffer���ͷ���buffer
			CameraReleaseImageBuffer(_hCamera, pbyBuffer);

			memcpy(&_sFrameInfo, &_sFrameInfo, sizeof(tSdkFrameHead));
		}
		else {
			LOG(ERROR) << "CameraGetImageBuffer status = " << status;
			throw_with_trace(std::runtime_error, "Failed to execute CameraGetImageBuffer");
		}
	}
	else {
		throw_with_trace(std::runtime_error, "Not initialized");
	}
}
