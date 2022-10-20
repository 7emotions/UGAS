#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- �ṩװ��ʶ����Ľӿ�
*/
#include "../GimbalSerial/GimbalSerial.h"
#include "NumberIdentifier/NumberIdentifier.h"
#include "../Common/DebugTools/DebugHeader.h"

class ArmorIdentifier {
protected:
	serial::GimbalSerial&	_com;
	NumberIdentifier&		_numberIdentifier;
public:
	ArmorIdentifier(serial::GimbalSerial& com, NumberIdentifier& numberIdentifier) :
		_com(com), _numberIdentifier(numberIdentifier)
		{ numberIdentifier.init(static_cast<void*>(&numberIdPara)); }

	virtual void Identify(const Img& img, std::vector<ArmorPlate>& result) = 0;
};
