#include "TargetSolution_V1.h"
using namespace cv;

cv::Vec3f TargetSolution_V1::SolvePNP(const ArmorPlate& armor) {
	Vec3f res;
	Mat rvec, tvec;
	if (isLargeArmor[armor.id]) {
		solvePnP(LargeArmor3f, armor.points, CameraMatrix, DistCoeffs,
			rvec, tvec, false, SOLVEPNP_AP3P);
	}
	else {
		solvePnP(NormalArmor3f, armor.points, CameraMatrix, DistCoeffs,
			rvec, tvec, false, SOLVEPNP_AP3P);
	}
	res = Vec3f(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
	// ����һ�����������ת��Ϊ��������ûд
	// ���������ǵ����ݲ�������ƫת

	return res;
}

void TargetSolution_V1::Solve(const std::vector<ArmorPlate>& armors) {
	// �����
	if (armors.size()) {
		_targets.clear();
		_targets.push_back(Target(SolvePNP(armors[0])));
	}
}
