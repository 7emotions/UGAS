#pragma once

#include <cmath>

#include <tuple>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "Control/Gimbal/Gimbal.h"
#include "Core/Transformer/SimpleTransformer.h"
#include "Util/Parameter/Parameters.h"
#include "Util/TimeStamp/TimeStampCounter.h"

class Trajectory_V1 {
public:
	/*! ��ȡ����Ƕȣ�����Ԥ�⴦��
	* \return ������̨ƫ��������ʽΪtuple[yaw, pitch]����λ�����ƣ���ѭ���ֶ���
	*/
	template <typename TransformerType>
	auto GetShotAngle(const Eigen::Vector3d& targetGimbalGyro, const double speed, const TransformerType& transformer) const {
		std::tuple<double, double> result;
		auto& [yaw, pitch] = result;

		Eigen::Vector3d shotVec = GetShotVector(transformer.GimbalGyro2MuzzleGyro(targetGimbalGyro), speed);
		shotVec = transformer.Gyro2Link(shotVec);

		yaw = atan2(shotVec.y(), shotVec.x());
		pitch = -atan2(shotVec.z(), sqrt(shotVec.y() * shotVec.y() + shotVec.x() + shotVec.x()));

		return result;
	}

	/*! ��ȡ����Ƕȣ���û��Ԥ�⡣
	* \return ������̨ƫ��������ʽΪtuple[yaw, pitch]����λ�����ƣ���ѭ���ֶ���
	*/
	template <typename TargetType, typename TransformerType>
	auto GetShotAngle(const TargetType& target, const double speed, const TransformerType& transformer) const {
		return GetShotAngle(target.Predict(0), speed, transformer);
	}


	template <typename TargetType>
	auto GetShotAngle(const TargetType& target, const double speed) const {
		auto pos = target.Predict(0);


		auto transformer = SimpleTransformer(pos);
		return GetShotAngle(transformer.CameraLink2GimbalLink(transformer.Link2Gyro(pos)), speed, transformer);
	}

private:
	Eigen::Vector3d GetShotVector(const Eigen::Vector3d& targetGimbalGyro, const double speed) const {
		// �����ǿ�������

		double x = targetGimbalGyro.x() / 1000;
		double y = targetGimbalGyro.y() / 1000;
		double z = targetGimbalGyro.z() / 1000;

		double hDis = sqrt(x * x + y * y);

		double yaw = atan2(y, x);
		double pitch = 0;

		double a = speed * speed;                  // v0 ^ 2
		double b = a * a;                          // v0 ^ 4
		double c = hDis * hDis;                    // xt ^ 2
		double d = c * c;                          // xt ^ 4
		double e = MathConsts::G * MathConsts::G;  // g ^ 2

		double f = b * d * (b - e * c - 2 * MathConsts::G * a * z);
		if (f >= 0) {
			pitch = -atan((b * c - sqrt(f)) / (MathConsts::G * a * c * hDis));
		}

		return { cos(pitch) * cos(yaw), cos(pitch) * sin(yaw),-sin(pitch) };
	}
};
