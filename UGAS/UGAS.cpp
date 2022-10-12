#include "UGAS.h"
using namespace std;
using namespace cv;

UGAS::UGAS() :
	_com(*new GIMBAL_SERIAL()),
	_imgCapture(*new IMG_CAPTURE())
{
	_com.Open(SERIAL_PORT);
	_com.RecvGimbalData();
	_imgCapture.init(&video);
	ParametersInit(static_cast<Team>(_com.team));
}

UGAS::~UGAS() {
	delete& _com;
	delete& _imgCapture;
}

void UGAS::initial() {
	try {
		// ��ʼ������

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
	while (true) {
		try {
			// ��Ҫ����ѭ��

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
