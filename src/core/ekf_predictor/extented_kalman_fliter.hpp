/**
 * @file extented_kalman_fliter.hpp
 * @author Lorenzo Feng (lorenzo.feng@njust.edu.cn)
 * @brief EKF to predict
 * @version 0.1
 * @date 2024-03-29
 *
 * @copyright Alliance, Nan Jing University of Science & Technology
 *
 */
#pragma once

#include <opencv2/core/types.hpp>
class EKFPredictor final {
public:
    EKFPredictor();

    void measure_update();
    [[nodiscard]] cv::Point3d predict() const;

private:
    class Impl;
};
