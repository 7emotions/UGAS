#pragma once
/*
Creation Date: 2022/11/20
Latest Update: 2022/11/20
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ���Ǹ������ģ��п�����������ʶ��������ֻ�᷵��NUM_DEFAULT
*/
#include"../NumberIdentifier.h"

class NullNumberIdentifier : public NumberIdentifier {
public:
	void init(void*) {}
	short Identify(const Img& img, const ArmorPlate& region) {
		return static_cast<short>(NUM_DEFAULT);
	}
};
