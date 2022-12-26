#include "Robot.h"
#include <Common/PnP/PnP.h>
#include <Parameters.h>
#include <Common/UniversalFunctions/UniversalFunctions.h>

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
			_movingSpeed = cv::Vec3f();
			_speedFilter.Reset();
		}
		else
		{ // �������ٵ�Ŀ��
			// �Ƚ�Ԥ�����۲��ľ��룬����Ԥ���
			cv::Point2f prediction = PnPsolver.RevertPnP(
				this->Predict(ImgTime - _latestUpdate)
			);

#if DEBUG_IMG == 1 && DEBUG_TRACK == 1
			cv::circle(debugImg, prediction, 5, COLOR_YELLOW, 2);
#endif

			// �����µ���ά����
			cv::Point3f lastPostion = _armorCenter;
			_armorCenter = PnPsolver.SolvePnP(armor);

			// ��ͬһ��װ�װ���Ч�ĸ���������ٶ�
			if (P2PDis(prediction, armor.center()) < maxArmorTrackDis ||
				// ���ڼ�����ٶȣ���ɲ��������Ӧ�Ż�
				P2PDis(_armor.center(), armor.center()) < maxArmorTrackDis ||
				// ���û���ٶ�ֱ�Ӹ��£��ӿ�Ա�����һ�����ٶ�Ŀ�����Ӧ
				_movingSpeed == cv::Vec3f())
			{ // �÷�װ�õ��˲�
				_movingSpeed = _speedFilter.Predict(
					static_cast<cv::Vec3f>(_armorCenter - lastPostion) /
					static_cast<double>(ImgTime - _latestUpdate)
				);
			}
			// ������������پ��룬����Ϊ��ͬһ��װ�װ壬�ڿɽ��ܵ�ʱ����ڼ̳��ٶ�
			// ����һ���϶�ʱ�伴�������ٶȵļ̳�
			else if (ImgTime - _latestUpdate > 100)
			{ // ���ֵ�����׵���������ʱŪ�ɶ�ֵ
				_movingSpeed = cv::Vec3f();
				_speedFilter.Reset();
			}
		}
		// ��ʱ�Ҵ�һ��
		_possibility = 100.;

		// ��������
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

	// û����Ч����ת��Ϣ��Ϊ����ģʽ
	if (_rotate == RotateDirc::UNKNOWN || // ˳���Ż�
		TimeStampCounter::GetTimeStamp() - _rotationLatestUpdate > rotation_validity * 1000)
	{
		prediction += _armorCenter;
	}
	else { // ����С���ݵ�Ԥ��
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

cv::Rect Robot::ROIregion(TimeStamp ImgTime) {
	if (_latestUpdate == ImgTime) {
		cv::Point2f center = _armor.center();
		cv::Size robotSize = cv::boundingRect(_armor.points).size();
		cv::Point2f ROIsize = cv::Point2f(robotSize.width * 1.5, robotSize.height) * 2.;

		cv::Point tl = center - ROIsize, br = center + ROIsize;
		if (tl.x < 0) tl.x = 0; if (tl.x >= frameWidth) tl.x = frameWidth - 1;
		if (tl.y < 0) tl.y = 0; if (tl.y >= frameHeight) tl.y = frameHeight - 1;
		if (br.x < 0) br.x = 0; if (br.x >= frameWidth) br.x = frameWidth - 1;
		if (br.y < 0) br.y = 0; if (br.y >= frameHeight) br.y = frameHeight - 1;

		return cv::Rect(tl, br);
	}
	return cv::Rect(0, 0, frameWidth, frameHeight);
}
