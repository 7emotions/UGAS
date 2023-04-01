#pragma once
/*
Creation Date: 2023/03/31
Latest Update: 2023/03/31
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- �������������ǣ������������ݺ�Ϊ0���������װ�װ�Ԥ��
- ʵ���ϲ�û��Ԥ�⣬����û�����κ���
*/


#include <vector>

#include <opencv2/opencv.hpp>

#include "Core/PnPSolver/ArmorPnPSolverInterface.h"
#include "Util/TimeStamp/TimeStampCounter.h"


class ArmorPredictor_V1 {
public:
	struct Target : ArmorPlate3d {
	public:
		cv::Point3f Predict(float sec) const {
			return position;
		}
		Target(const ArmorPlate3d& armor3d) : ArmorPlate3d(armor3d) { }
	};

	ArmorPredictor_V1() { }
	ArmorPredictor_V1(const ArmorPredictor_V1&) = delete;
	ArmorPredictor_V1(ArmorPredictor_V1&&) = delete;

	const std::vector<Target>& Update(const std::vector<ArmorPlate3d>& armors3d, TimeStamp timeStamp) {
		targets.clear();
		for (const ArmorPlate3d& armor3d : armors3d) {
			targets.emplace_back(armor3d);
		}
		return targets;
	}


private:
	std::vector<Target> targets;
};