#pragma once
/*
Creation Date: 2023/05/26
Latest Update: 2023/05/26
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
���ڵ������ԣ��ӿ���CBoardInfantry��ͬ������ʵ�ʲ�����
*/

#include "Util/Parameter/Parameters.h"

class VirtualCBoard {
public:
	VirtualCBoard(bool debugOutput = false) : _debugOutput(debugOutput) { }

	~VirtualCBoard() {	}

	/*! ����ڱ���ĵ�����ַ�����̨��׼����
	* \param yaw pitch ��λʹ�û����ƣ�������ѭ���ֶ���
	*/
	void Send(double yaw, double pitch) {
		if (_debugOutput) {
			yaw *= 180.0 / MathConsts::Pi;
			pitch *= 180.0 / MathConsts::Pi;
			std::cout << "Send: [" << yaw << ", " << pitch << "] deg\n";
		}
	}

	/*! �����˻�������̨��׼����
	* \param yaw pitch ��λʹ�û����ƣ�������ѭ���ֶ���
	*/
	void SendUAV(double yaw, double pitch) {
		if (_debugOutput) {
			yaw *= 180.0 / MathConsts::Pi;
			pitch *= 180.0 / MathConsts::Pi;
			std::cout << "SendUAV: [" << yaw << ", " << pitch << "] deg\n";
		}
	}

	void Receive() { }

	ArmorColor GetEnemyColor() {
		return Parameters::DefaultEnemyColor;
	}

	float GetBulletSpeed() {
		return Parameters::DefaultBulletSpeed;
	}

private:
	bool _debugOutput;
};