#include "UGAS.h"
using namespace std;
using namespace cv;

UGAS::UGAS() :
	_imgCapture(*new IMG_CAPTURE()),
	_pretreater(*new IMG_PRETREAT(com)),
	_armorIdentifier(*new ARMOR_IDENTIFY(*new NUMBER_IDENTIFY())),
	_targetSolution(*new TARGET_SOLUTION()),
	_trackingStrategy(*new TRACK_STRATEGY()),
	_trajectory(*new TRAJECTORY())
	{}

UGAS::~UGAS() {
	delete& _imgCapture;
	delete& _pretreater;
	delete& _armorIdentifier;
	delete& _targetSolution;
	delete& _trackingStrategy;
	delete& _trajectory;
}

void UGAS::initial() {
	try {
		// ��ʼ������
		// ͳһ���е�����Ĵ���
		PnPsolver._com = com;
		com.Get().Open(SERIAL_PORT);
		com.Get().RecvGimbalData();
		_imgCapture.init(&video);
		ParametersInit(static_cast<Team>(com.Get().team));
	}
	catch (const char* str) {
		throw_with_trace(std::runtime_error, str);
	}
}

void UGAS::always() {
	// �м���̱���
	Img					img;
	vector<ArmorPlate>	armors;
	int					targetID;
	double				yaw, pitch;

	while (true) {
		try {
			// ��Ҫ����ѭ��
			_imgCapture.read(img);
			_pretreater.GetPretreated(img);
			_armorIdentifier.Identify(img, armors);
			_targetSolution.Solve(img.timeStamp, armors);
			targetID = _trackingStrategy.GetTargetID();
			_trajectory.GetShotAngle(targetID, img.timeStamp, yaw, pitch);
			com.Get().SetAngle(yaw, pitch - com.Get().pitchA);
			com.Get().Send();

			_fps.Count();
			printf("\rNow time stamp:%llu | Fps: %3d     ",
				TimeStampCounter::GetTimeStamp(), _fps.GetFPS());
		}
		catch (const char* str) {
			throw_with_trace(std::runtime_error, str);
		}
	}
}
