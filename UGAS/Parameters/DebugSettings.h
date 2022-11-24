#pragma once 
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- �������е��Ժ꿪��

* ���Ժ�ʹ��ʾ�� (������)
* DEBUG_GRAGH(var)
* ���Ժ�ʹ��ʾ�� (�����)
* MAKE_GRAGH_DEFAULT
*     GRAGH_ADD_VAR(var1, COLOR_YELLOW)
*     GRAGH_ADD_VAR(var2, COLOR_BLUE)
* SHOW_GRAGH(Gragh_var1_var2)
*/

/// debug state switches ���Կ���
#define VIRTUAL_GIBAL	1					// ������̨����
#define DEBUG_IMG		1					// ��ʾ����ͼ���ܿ���
#define DEBUG_PARA		0					// ��̬�����϶�������
#define DEBUG_PRETREAT	0					// ʹ��Ԥ������ͼ����Ϊ����ͼ��
#define DEBUG_LIGHTBAR	0					// ���Ƶ���ʶ��ͼ��
#define DEBUG_ARMOR		1					// ����װ�װ�ʶ��ͼ��
#define DEBUG_ARMOR_NUM 1					// ����װ�װ�����ʶ����
#define DEBUG_TRACK		1					// ���Ƹ��ٵ�
#define DEBUG_PREDICT	1					// ����Ԥ���
#define DEBUG_ANGLE		0					// ����Yaw��Pitch����


/// static var			 ��̬����
#define VIDEO_VAR_TYPE	const char*			// ��Ƶ��������
#define VIDEO_VAR		"Blue_3.mp4" 		// ��Ƶ����
#define NUM_PARA_TYPE	const char*			// ����ʶ���������
#define NUM_PARA		"V1.pb"				// ����ʶ�����
#define FILTER_TYPE		PID::PDfilter		// Ŀ���˶��˲�������


/// debug var			 ���Ա���
#define DEFAULT_TEAM	Red 				// ������̨������ɫ
#define NUM_DEFAULT		1					// ����ʶ��Ĭ�Ϸ�������(һ����0)
#define DEFAULT_MEM_NUM 300					// ��������Ĭ������������


/// constant vars		 ��ֵ����
#define SERIAL_PORT		"\\\\.\\COM3"		// �����߼���
#define MAX_FPS			1000				// ���֡������
#define MAX_CNT			1000000				// �������������������
#define PI				3.1415926535897		// ��ֵ
#define G				9.8					// ��������


/// Time cost analysis	����ʱ������궨��
#define START_COUNT	{TimeStamp __tsTmp = TimeStampCounter::GetTimeStamp();
#define PRINT_COST	 printf("Took %llu ms | ", \
						TimeStampCounter::GetTimeStamp() - __tsTmp);
#define END_COUNT	}
#define	PRINT_END_COUNT PRINT_COST END_COUNT
