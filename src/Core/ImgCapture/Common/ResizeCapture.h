#pragma once
/*
Creation Date: 2023/03/21
Latest Update: 2023/03/21
Developer(s): 22-QZH
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- �ṩ������ImgCapture��ķ�װ����ԭCapture��Ļ��������Ӹ��Ĵ�С����
- ʹ�÷���(��CVVideoCapture��Ϊ��):
- ResizeCapture<CVVideoCapture> capture(0.5, "Blue_4.mp4");                //��ԭͼ��ȱ����ŵ�ԭ����0.5��
- ResizeCapture<CVVideoCapture> capture(cv::Size(640, 480), "Blue_4.mp4"); //��ԭͼ�����ŵ�(640, 480)��С
*/

#include <opencv2/opencv.hpp>

template<typename CaptureType>
class ResizeCapture : public CaptureType {
public:
	double _ratio;
	cv::Size _targetSize;

	ResizeCapture() = delete;

	template<typename... Types>
	ResizeCapture(double ratio, Types&&... args) : CaptureType(std::forward<Types>(args)...) {
		_ratio = ratio;
		_targetSize = cv::Size(0, 0);
	}

	template<typename... Types>
	ResizeCapture(cv::Size targetSize, Types&&... args) : CaptureType(std::forward<Types>(args)...) {
		_ratio = 0.0;
		_targetSize = targetSize;
	}

	ResizeCapture(const ResizeCapture&) = delete;
	ResizeCapture(ResizeCapture&&) = delete;


	std::tuple<cv::Mat, TimeStamp> Read() override {
		auto tuple = CaptureType::Read();
		auto& [mat, timestamp] = tuple;
		cv::resize(mat, mat, _targetSize, _ratio, _ratio);
		return tuple;
	}
};
