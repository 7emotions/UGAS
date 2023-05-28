#pragma once
/*
Creation Date: 2023/05/05
Latest Update: 2023/05/26
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
���˵��ӣ�HiPNUC���Ĺ��Ե����豸ͨѶ
*/

#include <chrono>
#include <thread>
#include <chrono>
#include <optional>

#include <Eigen/Dense>

#include "Util/Serial/SerialUtil.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Core/Transformer/IMUTransformer.h"

class HiPNUC {
public:
	HiPNUC(const char* portName) :
		_portName(portName),
		_destructed(false),
		_thread(&HiPNUC::_serialMain, this) {
	}
	HiPNUC(const HiPNUC&) = delete;
	HiPNUC(HiPNUC&&) = delete;

	~HiPNUC() {
		_destructed = true;
		_thread.join();
	}

	IMUTransformer GetTransformer() {
		const float* quatPtr = _transQuat;
		auto q = Eigen::Quaternionf{ quatPtr[0], quatPtr[1], quatPtr[2], quatPtr[3] };
		return IMUTransformer(q, _available);
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
		FPSCounter_V2 imuFps, errorFps;

		while (!_destructed) {
			try {

				serial::Serial serial(_portName, 115200, serial::Timeout::simpleTimeout(100));

				while (true) {
					if (_destructed) return;
					try {
						SerialUtil::SerialReceiver<DataD1, SerialUtil::Head<uint16_t, 0xa55a>, DataCRC16Calculator> receiver(serial);

						auto result = receiver.Receive();
						if (result == SerialUtil::ReceiveResult::Success) {
							const auto& data = receiver.GetReceivedData();

							// ÿ��GetReceivedData�õ������ݣ����������ڳ������´�Receive�ɹ����ٴε���Receiveǰ��
							// TODO: ����ʹ��ָ��洢��Ԫ��ʵ���������м�С�����ڶ�ȡʱ��ô������ݡ�
							_transQuat = &(data.quat[0]);

							if (imuFps.Count()) {
								_available = imuFps.GetFPS() > 100;
								std::cout << "HiPNUC IMU Fps: " << imuFps.GetFPS() << '\n';
							}
						}
						else if (result == SerialUtil::ReceiveResult::InvaildHeader)
							LOG(WARNING) << "HiPNUC: Invaild Header!";
						else if (result == SerialUtil::ReceiveResult::InvaildVerifyDegit)
							LOG(WARNING) << "HiPNUC: Invaild Verify Degit!";
					}
					catch (serial::IOException& e) {
						// windows�µĴ��ڻ�ż���Ե��׳�IOException���󲿷�ʱ����Ժ��Դ��쳣
						LOG(ERROR) << "HiPNUC: Unexpected serial::IOException: " << e.what();
						errorFps.Count();
						if (errorFps.GetFPS() > 10) throw std::runtime_error("HiPNUC: Exceptions are thrown too frequently.");
					}
					catch (serial::SerialException& e) {
						// linux�µĴ��ڻ��׳�SerialException��һ���ڴ��ڶϿ�����ʱ��������ʱ������᲻��ż�����׳�������ѡ����Դ��쳣
						LOG(ERROR) << "HiPNUC: Unexpected serial::SerialException: " << e.what();
						errorFps.Count();
						if (errorFps.GetFPS() > 10) throw std::runtime_error("HiPNUC: Exceptions are thrown too frequently.");
					}
				}
				
			}
			catch (serial::IOException& e) {
				LOG(ERROR) << "HiPNUC: Caught serial::IOException when serial init: " << e.what();
			}
			catch (std::exception& e) {
				LOG(ERROR) << "HiPNUC: Uncaught " << typeid(e).name() << ": " << e.what();
			}

			_available = false;
			LOG(ERROR) << "HiPNUC: Will reconnect in 1 second.";
			auto timingStart = std::chrono::steady_clock::now();
			while (std::chrono::steady_clock::now() - timingStart < std::chrono::seconds(1)) {
				if (_destructed) return;
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}
	}

	const char* _portName;
	std::atomic<bool> _destructed;
	std::thread _thread;

	std::atomic<bool> _available = false;

	const float _defaultTransQuat[4] = { 1, 0, 0, 0 };
	std::atomic<const float*> _transQuat = _defaultTransQuat;
};