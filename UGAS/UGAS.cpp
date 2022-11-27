#include "UGAS.h"
using namespace std;
using namespace cv;

UGAS::UGAS() :
	_imgCapture(*new IMG_CAPTURE()),
	_pretreater(*new IMG_PRETREAT()),
	_armorIdentifier(*new ARMOR_IDENTIFY(*new NUMBER_IDENTIFY())),
	_targetSolution(*new TARGET_SOLUTION()),
	_trackingStrategy(*new TRACK_STRATEGY()),
	_trajectory(*new TRAJECTORY())
	{}

UGAS::~UGAS() {
	delete& _imgCapture;
	delete& _pretreater;
	delete& _armorIdentifier;
	delete& _targetSolution;
	delete& _trackingStrategy;
	delete& _trajectory;
}

void UGAS::initial() {
	try {
		// ��ʼ������
		destroyAllWindows();
		com.Get().Open(SERIAL_PORT);
		com.Get().RecvGimbalData();
		_imgCapture.init(&video);
		ParametersInit(static_cast<Team>(com.Get().team));
	}
	catch (const char* str) { // �ذ�װ�쳣
		throw_with_trace(std::runtime_error, str);
	}
}

void UGAS::always() {
	// �м���̱���
	Img					img, imgThre, imgGray;
	Rect				ROIregion;
	vector<ArmorPlate>	armors;
	int					targetID;
	double				yaw, pitch;

	// ��ȡͼ���С
	_imgCapture.read(img);
	frameWidth = img.cols; frameHeight = img.rows;
	// ��ʼ��ROI
	ROIregion = Rect(0, 0, frameWidth, frameHeight);

	while (true) {
		try {
			/// =============== ��Ҫ����ѭ����ʼ ===============
			com.Get().RecvGimbalData();
			_imgCapture.read(img);

#if		DEBUG_IMG == 1 // ���ص������ͼ��
			debugImg.Load(img, ROIregion);
#endif	// DEBUG_IMG

#if ENABLE_ROI == 1 // ȡROI����
			img.SetROI(ROIregion);
			ROIoffset = ROIregion.tl();
#endif // ENABLE_ROI == 1

			_pretreater.GetPretreated(img, imgThre, imgGray);
			PnPsolver.GetTransMat();
			_armorIdentifier.Identify(imgThre, imgGray, armors);
			_targetSolution.Solve(img.timeStamp, armors);
			targetID = _trackingStrategy.GetTargetID();
			if (targetID) {
				_trajectory.GetShotAngle(targetID, img.timeStamp, yaw, pitch);
				com.Get().SetAngle(yaw, pitch);
			}
			else com.Get().SetAngle(yaw = .0, pitch = .0);
			com.Get().Send();

#if ENABLE_ROI == 1 // ����ROI
			if (targetID)
				ROIregion = robots[targetID].ROIregion(img.timeStamp);
			else ROIregion = Rect(0, 0, frameWidth, frameHeight);
#endif // ENABLE_ROI == 1

			/// =============== һЩ������Ϣ ===============
#if		DEBUG_PRETREAT == 1
			imshow("Pretreat", imgThre);
#endif

#if		DEBUG_ANGLE == 1
			MAKE_GRAGH_DEFAULT
				GRAGH_ADD_VAR(yaw, COLOR_YELLOW)
				GRAGH_ADD_VAR(pitch, COLOR_BLUE)
			SHOW_GRAGH(Gragh_Yaw_Pitch)
#endif // DEBUG_ANGLE == 1

#if		DEBUG_IMG == 1 //�����ͼ��
			_fps.Count();
			_fps.PrintFPS(debugImg);
			debugImg.Show();
#else		//ֱ�Ӵ�ӡ�ڿ���̨��
			printf("\rNow time stamp:%llu | Fps: %3d     ",
				TimeStampCounter::GetTimeStamp(), _fps.Count());
#endif	// DEBUG_IMG == 1
			/// =============== ��Ҫ����ѭ������ ===============
		}
		catch (const char* str) { // �ذ�װ�쳣
			throw_with_trace(std::runtime_error, str);
		}
	}
}
