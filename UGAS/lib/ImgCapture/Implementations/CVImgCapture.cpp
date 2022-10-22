#include "CVImgCapture.h"

void CVImgCapture::init(void* fileName) {
	open(*(const char**)fileName);
	if (!isOpened()) {
		open(std::string("resources/") + *(const char**)fileName);
		if (!isOpened())
			throw_with_trace(std::runtime_error, "Fail to open.");
	}
}

void CVImgCapture::read(Img& img) {
	VideoCapture::read(img);
	// ��֪��������ò�����
	img.timeStamp = get(cv::CAP_PROP_POS_MSEC);
}
