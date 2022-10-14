#include "CVImgCapture.h"

void CVImgCapture::init(void* fileName) {
	open(*(const char**)fileName);
	if (!isOpened())
		throw "<CVImgCapture::init> Fail to open.";
}

void CVImgCapture::read(Img& img) {
	VideoCapture::read(img);
	// ��֪��������ò�����
	img.timeStamp = get(cv::CAP_PROP_POS_MSEC);
}
