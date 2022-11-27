#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Class public functions:
- initial:
	Initialize for UGAS
- always:
	Working process in an endless loop
*/
#include "UGAS_Headers.h"

// main progress for UGAS
class UGAS {
private:
	// �����̹���ģ��
	ImgCapture&					_imgCapture;
	ImgPretreat&				_pretreater;
	ArmorIdentifier&			_armorIdentifier;
	TargetSolution&				_targetSolution;
	TrackingStrategy&			_trackingStrategy;
	Trajectory&					_trajectory;

	// ��������ģ��
	FPSCounter					_fps;
public:
	UGAS();
	virtual ~UGAS();

	void initial();
	void always();
};
