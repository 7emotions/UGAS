#include "Log.h"

INITIALIZE_EASYLOGGINGPP

void LOG_INIT() {
	try {
		//throw std::exception();
	}
	catch (std::exception& e) {
		std::cout << "Logger��ʼ��ʱ���ִ���" << typeid(e).name() << std::endl;
		std::cout << e.what() << std::endl;
		throw;
	}
}
