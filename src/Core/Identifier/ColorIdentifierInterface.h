#pragma once
/*
Creation Date: 2023/03/29
Latest Update: 2022/03/29
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- �Ե������ص������ɫʶ��Ľӿ�
- �������ɫ���Ŷȣ������� / ����-����0��ͨ�� / ����-����1��ͨ�� / ����-����2��ͨ�� / ����-����3��ͨ����
*/


enum class ColorConfidence : uchar {
	NotCredible = 0,
	CredibleZeroChannelOverexposure = 255,
	CredibleOneChannelOverexposure = 191,
	CredibleTwoChannelOverexposure = 127,
	CredibleThreeChannelOverexposure = 63
};


// ��������Ҫ��ColorIdentifier::Identify�����������ģ��ýӿڽ�������������̳�
class ColorIdentifierInterface final {
public:
	virtual ~ColorIdentifierInterface() = default;

	virtual ColorConfidence Identify(cv::Vec3b color) const = 0;
};
