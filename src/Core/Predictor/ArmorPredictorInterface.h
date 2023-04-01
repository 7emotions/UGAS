#pragma once
/*
Creation Date: 2023/03/31
Latest Update: 2023/03/31
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- װ�װ�Ԥ�����ӿ���
- ����Ϊ����װ�װ�����˱�������
*/

#include <vector>

#include "Core/PnPSolver/ArmorPnPSolverInterface.h"

class ArmorPredictorInterface final {
public:
	virtual ~ArmorPredictorInterface() = default;

	virtual void GetPredicted(const std::vector<ArmorPlate3d>&) = 0;
};