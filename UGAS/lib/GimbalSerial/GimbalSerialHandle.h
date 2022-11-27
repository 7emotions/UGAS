#pragma once
/*
Creation Date: 2022/11/19
Latest Update: 2022/11/19
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ��̨���ڵľ���࣬���������Ϊ�����࣬�����ʼ���븴������
*/
#ifdef _WIN32
#include <GimbalSerial/Implementations/Windows/WindowsGimbalSerial.h>
#define GIMBAL_SERIAL	serial::WindowsGimbalSerial
#else
#include <GimbalSerial/Implementations/Linux/LinuxGimbalSerial.h>
#define GIMBAL_SERIAL	serial::LinuxGimbalSerial
#endif

namespace serial {
	class GimbalSerialHandle {
	private:
		GimbalSerial* _com;
		int* _use; // ���ü���
	public:
		GimbalSerialHandle() : _use(new int(1)),
			_com(new GIMBAL_SERIAL()) {}
		GimbalSerialHandle(const GimbalSerialHandle& Hcom) {
			++* Hcom._use;
			_com = Hcom._com; _use = Hcom._use;
		}
		~GimbalSerialHandle() {
			if (-- * _use == 0) {
				delete _com;
				delete _use;
			}
		}

		GimbalSerialHandle& operator=(const GimbalSerialHandle& Hcom) {
			++* Hcom._use;
			if (-- * _use == 0) {
				delete _com;
				delete _use;
			}
			_com = Hcom._com; _use = Hcom._use;
			return *this;
		}
		operator GimbalSerial& () { return *_com; }

		GimbalSerial& Get() { return *_com; }
	};
}

extern serial::GimbalSerialHandle com;
