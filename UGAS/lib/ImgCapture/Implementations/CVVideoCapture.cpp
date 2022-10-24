#include "CVVideoCapture.h"

void CVVideoCapture::init(void* camIndex) {
	open(*(int*)camIndex);
	if (!isOpened())
		throw_with_trace(std::runtime_error, "Fail to open.");
}

void CVVideoCapture::read(Img& img) {
	VideoCapture::read(img);
	// ��֪��������ò�����
	img.timeStamp = get(cv::CAP_PROP_POS_MSEC);
}
