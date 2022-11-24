#include "TargetSolution_V1.h"
#include "Common/PnP/PnP.h"
#include "Common/UniversalFunctions/UniversalFunctions.h"
#include "Parameters.h"
using namespace cv;

inline bool TargetSolution_V1::isValidId(int id) {
	return 0 < id && id < 10;
}

void TargetSolution_V1::Solve(TimeStamp ImgTime, std::vector<ArmorPlate>& armors) {
	static std::vector<ArmorPlate> bin;

	bin.clear();
	for (auto& armor : armors) {
		if (isValidId(armor.id)) {
			robots[armor.id].Update(ImgTime, armor);
		}
		else
		{ // һ������Ͱ����
			bin.push_back(armor);
		}
	}
	if (bin.empty()) return;

	// ���Ի�������װ�װ�
	for (auto& robot : robots) {
		if (robot.LatestUpdate() != ImgTime &&
			ImgTime - robot.LatestUpdate() < keep_tracking * 1000)
		{ // һ������Ч��Ϣ��δ��ʱ���µĶ���
			// ��ȡ���άͼ��Ԥ���
			cv::Point2f prediction = PnPsolver.RevertPnP(
				robot.Predict(ImgTime - robot.LatestUpdate())
			);
			// ������
			for (auto& armor : bin) {
				if (P2PDis(armor.center(), prediction) < maxArmorTrackDis)
					(LOG(INFO) << "recycle!\n"),
					robot.Update(ImgTime, armor); // ���ճɹ�
			}
		}
	}
}
