#include "UGAS.h"
using namespace std;
using namespace cv;

UGAS::UGAS() :_com(*new GIMBAL_SERIAL()) {
	_com.Open(SERIAL_PORT);
	_com.RecvGimbalData();
	ParametersInit(static_cast<Team>(_com.team));
}

UGAS::~UGAS() {
	delete& _com;
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
