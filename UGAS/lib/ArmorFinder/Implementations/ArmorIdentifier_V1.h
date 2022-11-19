#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- �ṩһ����ʱ������ʵ�֣��ͽ� ArmorIdentifier Ver 1.0
*/
#include "../ArmorIdentifier.h"

class ArmorIdentifier_V1 : public ArmorIdentifier {
private:
	std::vector<LightBar> _lightBars;

	void FindLightBars(const Img& img);
	void FindArmorPlates(const Img& img, std::vector<ArmorPlate>& result);
public:
	ArmorIdentifier_V1(NumberIdentifier& numberIdentifier) :
		ArmorIdentifier(numberIdentifier) {}

	void Identify(const Img& img, std::vector<ArmorPlate>& result);
};
