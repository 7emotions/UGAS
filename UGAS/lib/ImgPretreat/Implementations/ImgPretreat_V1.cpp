#include "ImgPretreat_V1.h"
using namespace cv;

void ImgPretreat_V1::GetPretreated(Img& img) {
	frameWidth = img.cols; frameHeight = img.rows;
	if (img.empty())
		throw "[ERROR]<ImgPretreat> Get empty img in GetTreatedImg()!";
	cvtColor(img, img, COLOR_BGR2HSV);
	// �����ɫ������û�㣬�͵�������һ��Ū�ˣ����м�����������
	if (_com.team == ((int)Red ^ IN_STATE(_com.flag, STATE_BUFF)))
		inRange(img, Scalar(BHmin, BSmin, BVmin), Scalar(BHmax, BSmax, BVmax), img);
	else { // ��ɫHSV��Hɫ�ȸպ÷ֲ���ɫ������ͷ���÷ֿ�inRangeȻ��ϲ������鷳��
		Mat redL, redR;
		inRange(img, Scalar(RHminL, RSmin, RVmin), Scalar(RHmaxL, RSmax, RVmax), redL);
		inRange(img, Scalar(RHminR, RSmin, RVmin), Scalar(RHmaxR, RSmax, RVmax), redR);
		static_cast<Mat>(img) = redL | redR;
	}
	threshold(img, img, 0, 255, THRESH_BINARY);
#if DEBUG_PARA == 0
	static // �ǵ���ģʽ���þ�̬�ں�
#endif
		Mat closeCore = getStructuringElement(MORPH_RECT, Size(closeCoreSize | 1, closeCoreSize | 1));
	morphologyEx(img, img, MORPH_CLOSE, closeCore);
#if PRETREAT_DEBUG == 1
	imshow("Pretreated", _img);
	waitKey(1);
#endif
}
