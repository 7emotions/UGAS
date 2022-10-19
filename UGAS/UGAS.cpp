#include "UGAS.h"
using namespace std;
using namespace cv;

UGAS::UGAS() :
	_com(*new GIMBAL_SERIAL()),
	_imgCapture(*new IMG_CAPTURE()),
	_pretreater(*new IMG_PRETREAT(_com)),
	_armorIdentifier(*new ARMOR_IDENTIFY(_com, *new NUMBER_IDENTIFY())),
	_targetSolution(*new TARGET_SOLUTION(_com))
	{}

UGAS::~UGAS() {
	delete& _com;
	delete& _imgCapture;
	delete& _pretreater;
	delete& _armorIdentifier;
	delete& _targetSolution;
}

void UGAS::initial() {
	try {
		// ��ʼ������
		_com.Open(SERIAL_PORT);
		_com.RecvGimbalData();
		_imgCapture.init(&video);
		ParametersInit(static_cast<Team>(_com.team));
	}
	catch (const char* str) {
		cout << str;
		throw;
	}
	catch (...) {
		cout << "Unkown Error!";
		throw;
	}
}

void UGAS::always() {
	// �м���̱���
	Img						img;
	vector<ArmorPlate>		armors;
	const vector<Target>&	targets = _targetSolution.GetResultRefer();

	while (true) {
		try {
			// ��Ҫ����ѭ��
			_imgCapture.read(img);
			_pretreater.GetPretreated(img);
			_armorIdentifier.Identify(img, armors);
			_targetSolution.Solve(armors);

			_fps.Count();
			printf("\rNow time stamp:%llu | Fps: %3d     ",
				TimeStampCounter::GetTimeStamp(), _fps.GetFPS());
		}
		catch (const char* str) {
			cout << str;
			throw;
		}
		catch (...) {
			cout << "Unkown Error!";
			throw;
		}
	}
}
