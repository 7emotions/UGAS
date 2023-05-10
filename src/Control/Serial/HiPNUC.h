#pragma once
/*
Creation Date: 2023/05/05
Latest Update: 2023/05/05
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
���˵��ӣ�HiPNUC���Ĺ��Ե����豸ͨѶ
*/

#include <chrono>
#include <thread>

#include <Eigen/Dense>

#include "Util/Serial/SerialUtil.h"
#include "Util/FPSCounter/FPSCounter.h"

class HiPNUC {
public:
	HiPNUC(const char* portName) :
		_serial(portName, 115200, serial::Timeout::simpleTimeout(1000)),
		_receiver(_serial),
		_destructed(false),
		_thread(&HiPNUC::_serialMain, this) {
	}
	HiPNUC(const HiPNUC&) = delete;
	HiPNUC(HiPNUC&&) = delete;

	~HiPNUC() {
		_destructed = true;
		_thread.join();
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
				LOG(WARNING) << "HiPNUC: Invaild Header!";
			else if (result == SerialUtil::ReceiveResult::InvaildVerifyDegit)
				LOG(WARNING) << "HiPNUC: Invaild Verify Degit!";
		}
		if (received) {

		}
	}

	Eigen::Vector3f Transform(const Eigen::Vector3f& pos) const {
		const float* quatPtr = _transQuat;
		auto q = Eigen::Quaternionf{ quatPtr[0], quatPtr[1], quatPtr[2], quatPtr[3] };
		auto tranPos = q * Eigen::Quaternionf{0, pos.x() + 118.05f, -(pos.y() - 67.5f), pos.z() - 41.7f} * q.inverse();
		return { tranPos.x(), -tranPos.y(), tranPos.z() };
	}

private:
#pragma pack(push, 1)
	// ��׼���ݽṹ��
	struct Data91 {           // Unit      Name
		uint16_t length;      //       ֡��������ĳ��ȣ��̶�ֵ76
		uint16_t crc;         //       �����������������ֶε�CRC-16У���
		uint8_t tag;          //       ���ݰ���ǩ��0x91
		uint16_t useless;     //       ����
		uint8_t avg_temp;     //  ��C   ģ��������ƽ���¶�
		float pressure;       //  Pa   ��ѹ(�����ͺ�֧��)
		uint32_t timestamp;   //  ms   ������ʼ�ۼӵı���ʱ�����Ϣ��ÿ��������1
		float acc[3];         //  1G   ��������У׼��ļ��ٶȣ�˳��Ϊ��xyz
		float gyr[3];         // deg/s ��������У׼��Ľ��ٶȣ�˳��Ϊ��xyz
		float mag[3];         //  uT   ��ǿ�ȣ�˳��Ϊ��xyz
		float eul[3];         //  deg  ŷ���ǣ�˳��Ϊ��roll, pitch, yaw (yxz)
		float quat[4];        //       �ڵ���Ԫ�����ϣ�˳��Ϊ��wxyz
	};

	// ֻ����Ԫ�������ݽṹ��
	struct DataD1 {           // Unit      Name
		uint16_t length;      //       ֡��������ĳ��ȣ��̶�ֵ17
		uint16_t crc;         //       �����������������ֶε�CRC-16У���
		uint8_t tag1;         //       ���ݰ���ǩ��0xD1
		float quat[4];        //       �ڵ���Ԫ�����ϣ�˳��Ϊ��wxyz
	};
#pragma pack(pop)


	static constexpr int a = sizeof(Eigen::Quaternionf);

	class DataCRC16Calculator {
	public:
		using ResultType = SerialUtil::None;

		template <typename T>
		static bool Verify(const T& package) {
			static_assert(sizeof(T) == 82 || sizeof(T) == 23, "Wrong package size!");

			uint16_t checksum = 0x00;
			auto src = reinterpret_cast<const uint8_t*>(&package);

			_crc16Update(checksum, src, 4);
			_crc16Update(checksum, src + 6, sizeof(T) - 6);

			return checksum == *reinterpret_cast<const uint16_t*>(src + 4);
		}

	private:
		static void _crc16Update(uint16_t& currectCrc, const uint8_t* src, size_t lengthInBytes) {
			uint32_t crc = currectCrc;
			for (size_t j = 0; j < lengthInBytes; ++j) {
				uint32_t i;
				uint32_t byte = src[j];
				crc ^= byte << 8;
				for (i = 0; i < 8; ++i) {
					uint32_t temp = crc << 1;
					if (crc & 0x8000) {
						temp ^= 0x1021;
					}
					crc = temp;
				}
			}
			currectCrc = crc;
		}
	};

	void _serialMain() {
		double vx = 0, vy = 0, vz = 0;
		double x = 0, y = 0, z = 0;
		int interval = 100;

		auto fps = FPSCounter_V2();

		while (!_destructed) {
			auto result = _receiver.Receive();
			if (result == SerialUtil::ReceiveResult::Success) {
				const auto& data = _receiver.GetReceivedData();

				// ÿ��GetReceivedData�õ������ݣ����������ڳ������´�Receive�ɹ����ٴε���Receiveǰ��
				_transQuat = &(data.quat[0]);

				if (fps.Count()) {
					std::cout << "HiPNUC IMU Fps: " << fps.GetFPS() << '\n';
				}

				//auto q = Eigen::Quaterniond{ data.quat[0], data.quat[1], data.quat[2], data.quat[3] }.normalized();

				//auto euler = q.toRotationMatrix().eulerAngles(2, 0, 1);
				//float q2eul[3] = { euler.z() * 180 / MathConsts::Pi, euler.y() * 180 / MathConsts::Pi, euler.x() * 180 / MathConsts::Pi };
				/*auto acc = q * Eigen::Quaterniond{ 0, data.acc[0], data.acc[1], data.acc[2] } *q.inverse();
				double ax = acc.x(), ay = acc.y(), az = acc.z();
				if (ax < 0.1) ax = 0;
				if (ay < 0.1) ay = 0;
				if (az < 0.1) az = 0;

				vx += ax * MathConsts::G * 0.002, vy += ay * MathConsts::G * 0.002, vz += az * MathConsts::G * 0.002;

				x += vx * 0.002, y += vy * 0.002, z += vz * 0.002;

				if (abs(ay) > 0.1) {
					std::cout << ax << ' ' << ay << ' ' << az << '\n';
				}*/

			}
			else if (result == SerialUtil::ReceiveResult::InvaildHeader)
				LOG(WARNING) << "HiPNUC: Invaild Header!";
			else if (result == SerialUtil::ReceiveResult::InvaildVerifyDegit)
				LOG(WARNING) << "HiPNUC: Invaild Verify Degit!";
		}
	}

	serial::Serial _serial;
	SerialUtil::SerialReceiver<DataD1, SerialUtil::Head<uint16_t, 0xa55a>, DataCRC16Calculator> _receiver;
	std::atomic<bool> _destructed;
	std::thread _thread;

	const float _defaultTransQuat[4] = {1, 0, 0, 0};
	std::atomic<const float*> _transQuat = _defaultTransQuat;
};