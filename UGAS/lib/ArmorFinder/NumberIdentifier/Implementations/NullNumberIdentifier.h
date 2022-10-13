#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ���Ǹ������ģ��п�����������ʶ��������ֻ�᷵��0
*/
#include"../NumberIdentifier.h"

class NullNumberIdentifier : public NumberIdentifier {
public:
	void init(void*) {}
	short Identify(const Img& img, const ArmorPlate& region) {
		return static_cast<short>(0);
	}
};
