#pragma once
/*
Creation Date: 2022/10/19
Latest Update: 2022/10/19
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- �Ӷ��Ŀ����ѡ��һ���Ը���
  �ڶ�ʧĿ�����Ԥ����ٳ���
  ����ֵ��ʾ�Ƿ����ڸ���Ŀ��
*/
#include "Common/DebugTools/DebugHeader.h"

class TrackingStrategy {
public:
	virtual int GetTargetID() = 0;
};
