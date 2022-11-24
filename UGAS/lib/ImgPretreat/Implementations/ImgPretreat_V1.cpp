#include "ImgPretreat_V1.h"
using namespace cv;

void ImgPretreat_V1::GetPretreated(const cv::Mat& img, cv::Mat& imgThre, cv::Mat& imgGray) {
	if (img.empty())
		throw_with_trace(std::runtime_error, "Get empty img!");
	cvtColor(img, imgGray, COLOR_BGR2GRAY);
	cvtColor(img, imgThre, COLOR_BGR2HSV);
	// �����ɫ������û�㣬�͵�������һ��Ū�ˣ����м�����������
	if (com.Get().team == ((int)Red ^ IN_STATE(com.Get().flag, STATE_BUFF)))
		inRange(imgThre, Scalar(BHmin, BSmin, BVmin), Scalar(BHmax, BSmax, BVmax), imgThre);
	else { // ��ɫHSV��Hɫ�ȸպ÷ֲ���ɫ������ͷ���÷ֿ�inRangeȻ��ϲ������鷳��
		Mat tmp;
		inRange(imgThre, Scalar(0     , RSmin, RVmin), Scalar(RHmaxL, RSmax, RVmax), tmp);
		inRange(imgThre, Scalar(RHminR, RSmin, RVmin), Scalar(180   , RSmax, RVmax), imgThre);
		imgThre |= tmp;
	}
	threshold(imgThre, imgThre, 0, 255, THRESH_BINARY);

	//*///
#if DEBUG_PARA == 0
	static // �ǵ���ģʽ���þ�̬�ں�
#endif
		Mat closeCore = getStructuringElement(MORPH_RECT, Size(closeCoreSize | 1, closeCoreSize | 1));
	morphologyEx(imgThre, imgThre, MORPH_CLOSE, closeCore);
	//*///
}
