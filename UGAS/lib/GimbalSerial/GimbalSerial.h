#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/14
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved

�Ľ����������̫�����Դ����ʼ�����⣨C++��֧�ֿ����ã�
		����ʹ�ô����ü����ľ�������һ��

Class property:
- �̳��� SendPkg �� RecvPkg����ֱ������ת��ʹ��

Class public functions:
- Open		(const char* portName):
	����ΪportName�Ĵ���
- IsOpen	()
	��鴮���Ƿ���ã�����true��������false
- Close		()
	�رմ���

- Send			()
	�������� SendPkg ��������
- RecvGimbalData()
	�������ݵ����� RecvPkg ����
- GetGimbalData	()
	�������� RecvPkg ��������

- DebugGimbalInfo(std::ostream& os)
	���� SendPkg �� RecvPkg Debug�������
*/
#include <iostream>
#include "Packages.h"
#include "DebugSettings.h"
#include "Parameters.h"
#include "../Common/DebugTools/DebugHeader.h"

namespace serial {
	class GimbalSerial : public SendPkg, public RecvPkg {
	public:
		GimbalSerial()
		{ // ������̨�������ݳ�ʼ��
#if VIRTUAL_GIBAL == 1
			memset(this, 0, sizeof(*this));
			RecvPkg::head   = '\xFF';
			RecvPkg::team   = DEFAULT_TEAM;
			RecvPkg::yawA   = 0.f;
			RecvPkg::pitchA = 0.f;
			RecvPkg::rollA  = 0.f;
			RecvPkg::speed  = 30000.0;
			RecvPkg::flag   = STATE_NORMAL;
#endif
		}

		virtual void Open	(const char* portName)	= 0;
		virtual bool IsOpen	()						= 0;
		virtual void Close	()						= 0;

		virtual bool  Send						()	= 0;
		virtual const RecvPkg& RecvGimbalData	()	= 0;
		inline  const RecvPkg& GetGimbalData	() const
			{ return static_cast<const RecvPkg&>(*this); }

		virtual void DebugGimbalInfo(std::ostream& os) const {
			os << "Gimbal SendPkg:" << std::endl;
			this->SendPkg::Debug(os);
			os << "Gimbal RecvPkg:" << std::endl;
			this->RecvPkg::Debug(os);
		}
	};
}
