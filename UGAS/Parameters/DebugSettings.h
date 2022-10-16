#pragma once 
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- �������е��Ժ꿪��
*/

/// debug state switches ���Կ���
#define VIRTUAL_GIBAL	1					// ������̨����
#define DEBUG_PARA		1					// ��̬�����϶�������
#define DEBUG_PRETREAT	0					// ��ʾԤ������ͼ��
#define DEBUG_ARMOR		1					// ��ʾװ�װ�ʶ��ͼ��


/// static var			 ��̬����
#define VIDEO_VAR_TYPE	const char*			// ��Ƶ��������
#define VIDEO_VAR		"resources/Red.mp4"	// ��Ƶ����
#define NUM_PARA_TYPE	void*				// ����ʶ���������
#define NUM_PARA		nullptr				// ����ʶ�����


/// debug var			 ���Ա���
#define DEFAULT_TEAM	Blue				// ������̨������ɫ


/// constant vars		 ��ֵ����
#define SERIAL_PORT		"\\\\.\\COM3"		// �����߼���
#define MAX_FPS			1000				// ���֡�����ƣ����˻��׳��쳣haha��


/// Time cost analysis	����ʱ������궨��
#define START_COUNT {TimeStamp __tsTmp=TimeStampCounter::GetTimeStamp();
#define PRINT_COST printf("Took %llu ms | ", \
	TimeStampCounter::GetTimeStamp() - __tsTmp);
#define END_COUNT }
#define	PRINT_END_COUNT PRINT_COST END_COUNT


/// var(s) Debug func	�����������ɺ궨�� #


