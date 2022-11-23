#include "Robot.h"
#include "Common/PnP/PnP.h"
#include "Parameters.h"
#include "Common/UniversalFunctions/UniversalFunctions.h"

enum RotateDirc { UNKNOWN = 0, LEFT, RIGHT };

Robot robots[10];

Robot::Robot() :_latestUpdate(0ULL), _rotationLatestUpdate(0ULL),
	_robotCenter(), _movingSpeed(), _armor(), _armorCenter(),
	_rotate(RotateDirc::UNKNOWN), _rotateSpeed(.0), _possibility(.0) {}

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

			double passedTime = static_cast<double>(ImgTime - _latestUpdate);
			cv::Vec3f speed = static_cast<cv::Vec3f>(_armorCenter - lastPostion) / passedTime;

			if (VecLenth(speed - _movingSpeed) / passedTime > maxAcceleration)
				_speedFilter.Reset(); // ���������ٶȣ�����Ϊ��ͬһ��װ�װ�
			else { // ��ͬһ��װ�װ���Ч�ĸ���
				/* ��ʱ��һ�������˲�
				_movingSpeed = _movingSpeed * 0.97 + speed * 0.03;
				/*/// �����Ƿ�װ�õ��˲�
				_movingSpeed = _speedFilter.Predict(speed);
				//*/
			}
		}

		// ��ʱ�Ҵ�һ��
		_possibility = 100.;

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

double Robot::GetPossibility() {
	return (TimeStampCounter::GetTimeStamp() - _latestUpdate < keep_tracking * 1000) ? \
		_possibility : (_possibility = .0);
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


