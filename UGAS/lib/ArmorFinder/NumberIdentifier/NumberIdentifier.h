#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- �ṩ����ʶ����Ľӿ�
*/
#include <Common/UniversalStruct.h>

class NumberIdentifier {
public:
	virtual void init(void*) = 0;
	virtual short Identify(const Img& imgGray, const ArmorPlate& region) = 0;
};
