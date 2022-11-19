#include "ImgPretreat_V1.h"
using namespace cv;

void ImgPretreat_V1::GetPretreated(Img& img) {
	frameWidth = img.cols; frameHeight = img.rows;
	if (img.empty())
		throw_with_trace(std::runtime_error, "Get empty img!");
	cvtColor(img, img, COLOR_BGR2HSV);
	// �����ɫ������û�㣬�͵�������һ��Ū�ˣ����м�����������
	if (_com.Get().team == ((int)Red ^ IN_STATE(_com.Get().flag, STATE_BUFF)))
		inRange(img, Scalar(BHmin, BSmin, BVmin), Scalar(BHmax, BSmax, BVmax), img);
	else { // ��ɫHSV��Hɫ�ȸպ÷ֲ���ɫ������ͷ���÷ֿ�inRangeȻ��ϲ������鷳��
		Mat tmp;
		inRange(img, Scalar(RHminL, RSmin, RVmin), Scalar(RHmaxL, RSmax, RVmax), tmp);
		inRange(img, Scalar(RHminR, RSmin, RVmin), Scalar(RHmaxR, RSmax, RVmax), img);
		img |= tmp;
	}
	threshold(img, img, 0, 255, THRESH_BINARY);

#if DEBUG_PARA == 0
	static // �ǵ���ģʽ���þ�̬�ں�
#endif
		Mat closeCore = getStructuringElement(MORPH_RECT, Size(closeCoreSize | 1, closeCoreSize | 1));
	morphologyEx(img, img, MORPH_CLOSE, closeCore);
#if DEBUG_PRETREAT == 1
	imshow("Pretreated", img);
	waitKey(1);
#endif
}
