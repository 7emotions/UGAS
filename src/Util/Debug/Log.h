#pragma once
/*
Creation Date: 2022/10/17
Latest Update: 2022/10/18
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ����Easylogging++(https://github.com/amrayn/easyloggingpp)ͷ�ļ�
- �ṩthrow_with_trace�����׳�����ʱ�������λ����Ϣ
- �׳����쳣����̳���std:exception
- ÿ���׳��쳣ʱ��ͬ����¼��־(Log.Error)����˲�Ӧʹ���쳣�����Ƴ�����
*/

#include <exception>

#include "ThirdParty/EasyLogging++/EasyLogging++.h"


//�����__LINE_STR__������չ��Ϊ�������кŵ��ַ�����ʽ
#define __INNER_INT_TO_STR_1(R) #R
#define __INNER_INT_TO_STR_2(R) __INNER_INT_TO_STR_1(R)
#define __LINE_STR__ __INNER_INT_TO_STR_2(__LINE__)


//�����ƽ̨��__UNIVERSAL_FUNC__������չ��Ϊ�����ں�����
#ifdef __PRETTY_FUNCTION__
#define __UNIVERSAL_FUNC__ __PRETTY_FUNCTION__
#else
#ifdef __FUNCSIG__
#define __UNIVERSAL_FUNC__ __FUNCSIG__
#else
#ifdef __FUNCTION__
#define __UNIVERSAL_FUNC__ __FUNCTION__
#else
#ifdef __FUNC__
#define __UNIVERSAL_FUNC__ __FUNC__
#else
#ifdef __func__
#define __UNIVERSAL_FUNC__ __func__
#else
#define __UNIVERSAL_FUNC__ "unknown function"
#endif
#endif
#endif
#endif
#endif


// Ϊ�˺�throw���һ�£�������ȫ��д
// ʹ�ø�ʽ��throw std::xx_exception("��������"); ==> throw_with_trace(std:xx_exception, "��������");
// �����׳��Ĵ������ͱ���̳���std::exception���������ݱ���Ϊ�ַ���
#define throw_with_trace(exception_name, message) \
{ \
	exception_name exception_to_throw = exception_name(message); \
	LOG(ERROR) << "<" << typeid(exception_to_throw).name() << ("> thrown in file \"" __FILE__ "\", line " __LINE_STR__ ", at [" __UNIVERSAL_FUNC__ "]"); \
	throw exception_to_throw; \
}


// Ϊ�˺�throw���һ�£�������ȫ��д
// ʹ�ø�ʽ��
// catch (std::xx_exception e) {
//     log_trace(e);
//     ��Ĵ���
//     throw;
// } ��Ҫ�����׳����󣬱���ʹ��throw;����throw(e);�����߻ᵼ���쳣���Ͷ�ʧ��
#define log_trace(e) LOG(ERROR) << "<" << typeid(e).name() << ("> tracked in file \"" __FILE__ "\", line " __LINE_STR__ ", at [" __UNIVERSAL_FUNC__ "]");


// Ϊ�˺�throw���һ�£�������ȫ��д
// ʹ�ø�ʽ��
// catch (...) {
//     log_trace_unknown();
//     ��Ĵ���
//     throw;
// } ��Ҫ�����׳����󣬱���ʹ��throw;����throw(e);�����߻ᵼ���쳣���Ͷ�ʧ��
#define log_trace_unknown() LOG(ERROR) << ("<UNKNOWN EXCEPTION> tracked in file \"" __FILE__ "\", line " __LINE_STR__ ", at [" __UNIVERSAL_FUNC__ "]");
