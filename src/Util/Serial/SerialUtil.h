#pragma once
/*
Creation Date: 2023/04/21
Latest Update: 2023/05/05
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ���ڹ���
*/

#include <cstdint>

#include "ThirdParty/Serial/serial.h"
#include "Util/Serial/CRC.h"
#include "Util/Debug/Log.h"

namespace SerialUtil {

	void PrintByteArray(const uint8_t* array, size_t size) {
		std::cout << std::hex << std::setfill('0');
		for (size_t i = 0; i < size; ++i)
			std::cout << std::setw(2) << static_cast<int>(array[i]) << ' ';
		std::cout << std::dec << '\n';
	}

	template <typename T>
	void PrintStructAsByteArray(const T& data) {
		PrintByteArray(reinterpret_cast<const uint8_t*>(&data), sizeof(T));
	}

	// ��Ǹ������
	struct None { };

#pragma pack(push, 1)
	template <typename HeadDataType, HeadDataType HeadValue>
	struct Head {
		HeadDataType data;

		// ����ֵ��ȷ
		void MakeCorrect() {
			data = HeadValue;
		}

		// ����ֵ����ȷ
		void MakeIncorrect() {
			data = ~HeadValue;
		}

		// �ж�ֵ�Ƿ���ȷ
		bool IsCorrect() {
			return data == HeadValue;
		}
	};

	template <typename DataType, typename HeadType, typename ChecksumResultType>
	struct Package {
		HeadType head;
		DataType data;
		ChecksumResultType crc;
	};

	template <typename DataType>
	struct Package<DataType, None, None> {
		DataType data;
	};

	template <typename DataType, typename HeadType>
	struct Package<DataType, HeadType, None> {
		HeadType head;
		DataType data;
	};

	template <typename DataType, typename ChecksumResultType>
	struct Package<DataType, None, ChecksumResultType> {
		DataType data;
		ChecksumResultType crc;
	};
#pragma pack(pop)


	// ��ͷ��У��Ĵ������ݰ�������
	template <typename DataType, typename HeadType, typename ChecksumCalculatorType>
	class SerialSender {
	public:
		SerialSender(serial::Serial& serial) : _serial(serial) {
			_pkg.head.MakeCorrect();
		}
		SerialSender(const SerialSender&) = delete;
		SerialSender(SerialSender&&) = delete;

		void Send() {
			ChecksumCalculatorType::Append(_pkg);
			_serial.write(reinterpret_cast<uint8_t*>(&_pkg), sizeof(_pkg));
		}

		DataType& Data = _pkg.data;
	private:
		serial::Serial& _serial;
		Package<DataType, HeadType, typename ChecksumCalculatorType::ResultType> _pkg;
	};

	// ��ͷ��У��Ĵ������ݰ�������
	template <typename DataType>
	class SerialSender<DataType, None, None> {
	public:
		SerialSender(serial::Serial& serial) : _serial(serial) { }
		SerialSender(const SerialSender&) = delete;
		SerialSender(SerialSender&&) = delete;

		void Send() {
			_serial.write(reinterpret_cast<uint8_t*>(&_pkg), sizeof(_pkg));
		}

		DataType& Data = _pkg.data;
	private:
		serial::Serial& _serial;
		Package<DataType, None, None> _pkg;
	};

	// ��ͷ��У��Ĵ������ݰ�������
	template <typename DataType, typename HeadType>
	class SerialSender<DataType, HeadType, None> {
	public:
		SerialSender(serial::Serial& serial) : _serial(serial) {
			_pkg.head.MakeCorrect();
		}
		SerialSender(const SerialSender&) = delete;
		SerialSender(SerialSender&&) = delete;

		void Send() {
			_serial.write(reinterpret_cast<uint8_t*>(&_pkg), sizeof(_pkg));
		}

		DataType& Data = _pkg.data;
	private:
		serial::Serial& _serial;
		Package<DataType, HeadType, None> _pkg;
	};

	// ��ͷ��У��Ĵ������ݰ�������
	template <typename DataType, typename ChecksumCalculatorType>
	class SerialSender<DataType, None, ChecksumCalculatorType> {
	public:
		SerialSender(serial::Serial& serial) : _serial(serial) { }
		SerialSender(const SerialSender&) = delete;
		SerialSender(SerialSender&&) = delete;

		void Send() {
			ChecksumCalculatorType::Append(_pkg);
			_serial.write(reinterpret_cast<uint8_t*>(&_pkg), sizeof(_pkg));
		}

		DataType& Data = _pkg.data;
	private:
		serial::Serial& _serial;
		Package<DataType, None, typename ChecksumCalculatorType::ResultType> _pkg;
	};


	enum class ReceiveResult : uint8_t {
		Success = 0, Timeout = 1, InvaildHeader = 2, InvaildVerifyDegit = 4
	};

	// ��ͷ��У��Ĵ������ݰ�������
	template <typename DataType, typename HeadType, typename ChecksumCalculatorType>
	class SerialReceiver {
	public:
		SerialReceiver(serial::Serial& serial) : _serial(serial) { }
		SerialReceiver(const SerialReceiver&) = delete;
		SerialReceiver(SerialReceiver&&) = delete;

		/*! �������һ�����ݣ��������κ����ݣ���У��δͨ��֮�⡣�����ڽ��շ���Ƶ�ʽϵ͵����ݡ�
		* �÷��������������������ÿ�ε��ø÷���ֻ�����һ��serial.read����˳�ʱʱ��ȡ����Serial������ĳ�ʱʱ���趨������Ϊ0��������
		* ʹ��GetCacheSize�������Ի�ȡ�ѻ������ݵĳ��ȣ�ʹ��ClearCache��������ǿ����ջ�������ݡ�
		*
		* \return �ɹ�ʱ������Success
		* ��δ���յ��㹻�������ݣ����´��ڽ��ճ�ʱʱ���÷�������Timeout�����´ε��ø÷���ʱ�᳢�Խ���ʣ�ಿ�֡�
		* �����յ������ݰ�ͷδ���룬��У��δͨ�����÷�������InvaildHeader��InvaildVerifyDegit�������Ѱ��ƥ���ͷ�����´ε��ø÷���ʱ���Խ���ʣ�ಿ�֡�
		*/
		ReceiveResult Receive() {
			ReceiveResult result;
			// ���վ����ܶ������
			_cacheSize += _serial.read(reinterpret_cast<uint8_t*>(&_receivePkg->head) + _cacheSize, sizeof(PackageType) - _cacheSize);

			if (_cacheSize >= sizeof(HeadType)) {                 // �ɹ����յ���ͷ����
				if (_receivePkg->head.IsCorrect()) {               // ���ݰ�ͷ��ȷ
					if (_cacheSize == sizeof(PackageType)) {    // ����ͷ��ȷ�����ݽ������������Խ�һ��У��
						if (ChecksumCalculatorType::Verify(*_receivePkg)) {
							_cacheSize = 0;
							std::swap(_receivePkg, _resultPkg);
							return ReceiveResult::Success;
						}
						result = ReceiveResult::InvaildVerifyDegit;
					}
					else return ReceiveResult::Timeout;                         // ����ͷ��ȷ������δ�����������򷵻�Timeout���ȴ���һ�ν���
				}
				else result = ReceiveResult::InvaildHeader;

				// �����ݰ�ͷ���������Ѱ��ƥ���ͷ�����ҵ����������������ǰ���룬�Ա���һ�ν���ʣ�ಿ��
				// ��ʹû���ҵ�ƥ���ͷ��������ʣ���ֽ�������ͷ���ֽ���ʱ��Ҳ���ʣ���ֽ���ǰ����
				--_cacheSize;
				uint8_t* bufferHead = reinterpret_cast<uint8_t*>(&_receivePkg->head) + 1;

				while (true) {
					if (_cacheSize < sizeof(HeadType) || reinterpret_cast<HeadType*>(bufferHead)->IsCorrect()) {
						for (size_t i = 0; i < _cacheSize; ++i)
							reinterpret_cast<uint8_t*>(&_receivePkg->head)[i] = bufferHead[i];
						break;
					}
					--_cacheSize;
					++bufferHead;
				}
				return result;
			}
			else return ReceiveResult::Timeout;                   // ������ͷ��û�ӵ���Ҳ����Timeout
		}

		/* ��ȡ���յ������� */
		const DataType& GetReceivedData() { return _resultPkg->data; }

		/* ��ȡ��ǰ��������ݳ��� */
		size_t GetCacheSize() { return _cacheSize; }

		/* ���õ�ǰ���ջ��� */
		size_t ClearCache() { _cacheSize = 0; }

	private:
		serial::Serial& _serial;
		size_t _cacheSize = 0;  // ��ǻ�������ݳ���

		using PackageType = Package<DataType, HeadType, typename ChecksumCalculatorType::ResultType>;
		// _receivePkgָ����ջ�����, _resultPkgָ����һ���ѽ��ղ���֤ͨ���Ļ�����
		PackageType _pkg[2], * _receivePkg = &_pkg[0], * _resultPkg = &_pkg[1];
	};

	// ��ͷ��У��Ĵ������ݰ�������
	template <typename DataType>
	class SerialReceiver<DataType, None, None> {
	public:
		SerialReceiver(serial::Serial& serial) : _serial(serial) { }
		SerialReceiver(const SerialReceiver&) = delete;
		SerialReceiver(SerialReceiver&&) = delete;

		/*! �������һ�����ݣ��������κ����ݣ���У��δͨ��֮�⡣�����ڽ��շ���Ƶ�ʽϵ͵����ݡ�
		* �÷��������������������ÿ�ε��ø÷���ֻ�����һ��serial.read����˳�ʱʱ��ȡ����Serial������ĳ�ʱʱ���趨������Ϊ0��������
		* ʹ��GetCacheSize�������Ի�ȡ�ѻ������ݵĳ��ȣ�ʹ��ClearCache��������ǿ����ջ�������ݡ�
		*
		* \return �ɹ�ʱ������Success
		* ��δ���յ��㹻�������ݣ����´��ڽ��ճ�ʱʱ���÷�������Timeout�����´ε��ø÷���ʱ�᳢�Խ���ʣ�ಿ�֡�
		* �����յ������ݰ�ͷδ���룬��У��δͨ�����÷�������InvaildHeader��InvaildVerifyDegit�������Ѱ��ƥ���ͷ�����´ε��ø÷���ʱ���Խ���ʣ�ಿ�֡�
		*/
		ReceiveResult Receive() {
			_cacheSize += _serial.read(reinterpret_cast<uint8_t*>(_receivePkg) + _cacheSize, sizeof(DataType) - _cacheSize);

			if (_cacheSize == sizeof(DataType)) {
				_cacheSize = 0;
				std::swap(_receivePkg, _resultPkg);
				return ReceiveResult::Success;
			}
			else return ReceiveResult::Timeout;
		}

		/* ��ȡ���յ������� */
		const DataType& GetReceivedData() { return _resultPkg->data; }

		/* ��ȡ��ǰ��������ݳ��� */
		size_t GetCacheSize() { return _cacheSize; }

		/* ���õ�ǰ���ջ��� */
		size_t ClearCache() { _cacheSize = 0; }

	private:
		serial::Serial& _serial;
		size_t _cacheSize = 0;  // ��ǻ�������ݳ���
		Package<DataType, None, None> _pkg[2], * _receivePkg = &_pkg[0], * _resultPkg = &_pkg[1];
	};

	// ��ͷ��У��Ĵ������ݰ�������
	template <typename DataType, typename HeadType>
	class SerialReceiver<DataType, HeadType, None> {
	public:
		SerialReceiver(serial::Serial& serial) : _serial(serial) { }
		SerialReceiver(const SerialReceiver&) = delete;
		SerialReceiver(SerialReceiver&&) = delete;

		/*! �������һ�����ݣ��������κ����ݣ���У��δͨ��֮�⡣�����ڽ��շ���Ƶ�ʽϵ͵����ݡ�
		* �÷��������������������ÿ�ε��ø÷���ֻ�����һ��serial.read����˳�ʱʱ��ȡ����Serial������ĳ�ʱʱ���趨������Ϊ0��������
		* ʹ��GetCacheSize�������Ի�ȡ�ѻ������ݵĳ��ȣ�ʹ��ClearCache��������ǿ����ջ�������ݡ�
		*
		* \return �ɹ�ʱ������Success
		* ��δ���յ��㹻�������ݣ����´��ڽ��ճ�ʱʱ���÷�������Timeout�����´ε��ø÷���ʱ�᳢�Խ���ʣ�ಿ�֡�
		* �����յ������ݰ�ͷδ���룬��У��δͨ�����÷�������InvaildHeader��InvaildVerifyDegit�������Ѱ��ƥ���ͷ�����´ε��ø÷���ʱ���Խ���ʣ�ಿ�֡�
		*/
		ReceiveResult Receive() {
			ReceiveResult result;
			// ���վ����ܶ������
			_cacheSize += _serial.read(reinterpret_cast<uint8_t*>(&_receivePkg->head) + _cacheSize, sizeof(HeadType) + sizeof(DataType) - _cacheSize);

			if (_cacheSize >= sizeof(HeadType)) {                 // �ɹ����յ���ͷ����
				if (_receivePkg->head.IsCorrect()) {               // ���ݰ�ͷ��ȷ
					if (_cacheSize == sizeof(HeadType) + sizeof(DataType)) {    // ����ͷ��ȷ�����ݽ����������򷵻�Success
						_cacheSize = 0;
						std::swap(_receivePkg, _resultPkg);
						return ReceiveResult::Success;
					}
					else return ReceiveResult::Timeout;                         // ����ͷ��ȷ������δ�����������򷵻�Timeout���ȴ���һ�ν���
				}
				else result = ReceiveResult::InvaildHeader;

				// �����ݰ�ͷ���������Ѱ��ƥ���ͷ�����ҵ����������������ǰ���룬�Ա���һ�ν���ʣ�ಿ��
				// ��ʹû���ҵ�ƥ���ͷ��������ʣ���ֽ�������ͷ���ֽ���ʱ��Ҳ���ʣ���ֽ���ǰ����
				--_cacheSize;
				uint8_t* bufferHead = reinterpret_cast<uint8_t*>(&_receivePkg->head) + 1;

				while (true) {
					if (_cacheSize < sizeof(HeadType) || reinterpret_cast<HeadType*>(bufferHead)->IsCorrect()) {
						for (size_t i = 0; i < _cacheSize; ++i)
							reinterpret_cast<uint8_t*>(&_receivePkg->head)[i] = bufferHead[i];
						break;
					}
					--_cacheSize;
					++bufferHead;
				}
				return result;
			}
			else return ReceiveResult::Timeout;                   // ������ͷ��û�ӵ���Ҳ����Timeout
		}

		/* ��ȡ���յ������� */
		const DataType& GetReceivedData() { return _resultPkg->data; }

		/* ��ȡ��ǰ��������ݳ��� */
		size_t GetCacheSize() { return _cacheSize; }

		/* ���õ�ǰ���ջ��� */
		size_t ClearCache() { _cacheSize = 0; }

	private:
		serial::Serial& _serial;
		size_t _cacheSize = 0;  // ��ǻ�������ݳ���

		// _receivePkgָ����ջ�����, _resultPkgָ����һ���ѽ��ղ���֤ͨ���Ļ�����
		Package<DataType, HeadType, None> _pkg[2], * _receivePkg = &_pkg[0], * _resultPkg = &_pkg[1];

	};

	// ��ͷ��У��Ĵ������ݰ�������
	template <typename DataType, typename ChecksumCalculatorType>
	class SerialReceiver<DataType, None, ChecksumCalculatorType> {
	public:
		SerialReceiver(serial::Serial& serial) : _serial(serial) { }
		SerialReceiver(const SerialReceiver&) = delete;
		SerialReceiver(SerialReceiver&&) = delete;

		/*! �������һ�����ݣ��������κ����ݣ���У��δͨ��֮�⡣�����ڽ��շ���Ƶ�ʽϵ͵����ݡ�
		* �÷��������������������ÿ�ε��ø÷���ֻ�����һ��serial.read����˳�ʱʱ��ȡ����Serial������ĳ�ʱʱ���趨������Ϊ0��������
		* ʹ��GetCacheSize�������Ի�ȡ�ѻ������ݵĳ��ȣ�ʹ��ClearCache��������ǿ����ջ�������ݡ�
		*
		* \return �ɹ�ʱ������Success
		* ��δ���յ��㹻�������ݣ����´��ڽ��ճ�ʱʱ���÷�������Timeout�����´ε��ø÷���ʱ�᳢�Խ���ʣ�ಿ�֡�
		* �����յ������ݰ�ͷδ���룬��У��δͨ�����÷�������InvaildHeader��InvaildVerifyDegit�������Ѱ��ƥ���ͷ�����´ε��ø÷���ʱ���Խ���ʣ�ಿ�֡�
		*/
		ReceiveResult Receive() {
			// ���վ����ܶ������
			_cacheSize += _serial.read(reinterpret_cast<uint8_t*>(&_receivePkg->data) + _cacheSize,
				sizeof(DataType) + sizeof(typename ChecksumCalculatorType::ResultType) - _cacheSize);

			if (_cacheSize == sizeof(DataType) + sizeof(typename ChecksumCalculatorType::ResultType)) {    // �����ݽ������������Խ�һ��У��
				if (ChecksumCalculatorType::Verify(*_receivePkg)) {
					// У��ͨ���򷵻�Success
					_cacheSize = 0;
					std::swap(_receivePkg, _resultPkg);
					return ReceiveResult::Success;
				}
				else {
					// ��У�鲻ͨ�����������������ǰ����һλ����һ�ν���ʣ�ಿ��
					--_cacheSize;
					uint8_t* bufferHead = reinterpret_cast<uint8_t*>(&_receivePkg->data) + 1;
					for (size_t i = 0; i < _cacheSize; ++i)
						reinterpret_cast<uint8_t*>(&_receivePkg->data)[i] = bufferHead[i];
					return ReceiveResult::InvaildVerifyDegit;
				}
			}

			// ������δ�����������򷵻�Timeout���ȴ���һ�ν���
			return ReceiveResult::Timeout;
		}

		/* ��ȡ���յ������� */
		const DataType& GetReceivedData() { return _resultPkg->data; }

		/* ��ȡ��ǰ��������ݳ��� */
		size_t GetCacheSize() { return _cacheSize; }

		/* ���õ�ǰ���ջ��� */
		size_t ClearCache() { _cacheSize = 0; }

	private:
		serial::Serial& _serial;
		size_t _cacheSize = 0;  // ��ǻ�������ݳ���

		// _receivePkgָ����ջ�����, _resultPkgָ����һ���ѽ��ղ���֤ͨ���Ļ�����
		Package<DataType, None, typename ChecksumCalculatorType::ResultType> _pkg[2], * _receivePkg = &_pkg[0], * _resultPkg = &_pkg[1];
	};

}
