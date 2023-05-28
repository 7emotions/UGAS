#pragma once
/*
Creation Date: Unknown
Latest Update: 2023/04/21
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
�벽���Ĵ���ͨѶ
*/

#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

#include "Control/Gimbal/Gimbal.h"
#include "Util/Serial/CRC.h"
#include "Util/Serial/SerialUtil.h"
#include "Util/Debug/Log.h"
#include "Util/Parameter/Parameters.h"

class CBoardInfantry {
public:

#pragma pack(push, 1)
	struct DataSend {
		float yaw, pitch;
	};
	struct DataReceive {
		uint8_t selfColor;             // ���������ɫ��1-�죬2-��
		uint8_t presetBulletSpeed;     // Ԥ�赯�٣���λ��m/s
		float bulletSpeed;             // ʵʱ���٣���λ��m/s
	};
#pragma pack(pop)

	CBoardInfantry(const char* portName) :
		_serial(portName, 115200, serial::Timeout::simpleTimeout(0)),
		_sender(_serial),
		_receiver(_serial) {
	}

	~CBoardInfantry() {	}

	/*! ����ڱ���ĵ�����ַ�����̨��׼����
	* \param yaw pitch ��λʹ�û����ƣ�������ѭ���ֶ���
	*/
	void Send(double yaw, double pitch) {
		_sender.Data.yaw = -yaw * 180.0 / MathConsts::Pi;
		_sender.Data.pitch = -pitch * 180.0 / MathConsts::Pi;
		_sender.Send();
	}

	/*! �����˻�������̨��׼����
	* \param yaw pitch ��λʹ�û����ƣ�������ѭ���ֶ���
	*/
	void SendUAV(double yaw, double pitch) {
		_sender.Data.yaw = -yaw;
		_sender.Data.pitch = -pitch;
		_sender.Send();
	}
	
	void Receive() {
		bool received = false;

		while (true) {
			auto result = _receiver.Receive();
			if (result == SerialUtil::ReceiveResult::Success)
				received = true;
			else if (result == SerialUtil::ReceiveResult::Timeout)
				break;
			else if (result == SerialUtil::ReceiveResult::InvaildHeader)
				LOG(WARNING) << "CboardInfantry: Invaild Header!";
			else if (result == SerialUtil::ReceiveResult::InvaildVerifyDegit)
				LOG(WARNING) << "CboardInfantry: Invaild Verify Degit!";
		}

		if (received) {
			const auto& data = _receiver.GetReceivedData();

			if (data.selfColor == 1)        // ������ɫ��������ɫ
				_enemyColor = ArmorColor::Blue;
			else if (data.selfColor == 2)   // ������ɫ�������ɫ
				_enemyColor = ArmorColor::Red;

			_bulletSpeed = data.presetBulletSpeed;  // ��ʱ������ʵʱ����
		}
	}

	ArmorColor GetEnemyColor() {
		return _enemyColor;
	}

	float GetBulletSpeed() {
		return _bulletSpeed;
	}

private:
	serial::Serial _serial;

	SerialUtil::SerialSender<DataSend, SerialUtil::Head<uint8_t, 0xff>, CRC::DjiCRC8Calculator> _sender;
	SerialUtil::SerialReceiver<DataReceive, SerialUtil::Head<uint8_t, 0xff>, CRC::DjiCRC8Calculator> _receiver;
	ArmorColor _enemyColor = Parameters::DefaultEnemyColor;
	float _bulletSpeed = Parameters::DefaultBulletSpeed;

};