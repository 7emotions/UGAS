#include "Robot.h"
#include "Common/PnP/PnP.h"
#include "Parameters.h"

enum RotateDirc { UNKNOWN = 0, LEFT, RIGHT };

Robot robots[10];

Robot::Robot() :_latestUpdate(0ULL), _rotationLatestUpdate(0ULL),
	_robotCenter(), _movingSpeed(), _armor(),
	_rotate(RotateDirc::UNKNOWN), _rotateSpeed(.0) {}

void Robot::Update(TimeStamp ImgTime, const ArmorPlate& armor) {
	if (_latestUpdate != ImgTime) {
		if (ImgTime - _latestUpdate > keep_tracking * 1000)
		{ // �¹۲쵽��Ŀ��
			_armorCenter = PnPsolver.SolvePnP(armor);
		}
		else
		{ // �������ٵ�Ŀ��
			cv::Point3f lastPostion = _armorCenter;
			_armorCenter = PnPsolver.SolvePnP(armor);
			// ��ʱ��һ�������˲�
			_movingSpeed = _movingSpeed * 0.9 +
				static_cast<cv::Vec3f>(_armorCenter - lastPostion) / 
				static_cast<double>(ImgTime - _latestUpdate) * 0.1;
		}

		_armor = armor;
		_latestUpdate = ImgTime;
	}
	else { // ���¹۲�ĵڶ���װ�װ�
		if (ImgTime - _rotationLatestUpdate > rotation_validity * 1000)
		{ // �µõ�����ת����

		}
		else
		{ // �������µ���ת����

		}

		_rotationLatestUpdate = ImgTime;
	}
}

cv::Point3f Robot::Predict(int millisec) const {
	cv::Point3f prediction = _movingSpeed * millisec;

	if (_rotate == RotateDirc::UNKNOWN) {
		prediction += _armorCenter;
	}
	else {
		prediction += _robotCenter;
		switch (_rotate) {
		case LEFT:

			break;
		case RIGHT:

			break;
		default: break;
		}
	}
	return prediction;
}


