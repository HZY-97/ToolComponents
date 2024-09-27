/**
 * @file share_data_test.hpp
 * @author huizeyu (huizeyu@siasun.com)
 * @brief
 * @version 0.1
 * @date 2024-09-26
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <iostream>

#include "transform3d.h"

using namespace std;

const string seg = "-----------------------------------\n";

const double deg_45 = 45.0;
const double rad_45 = 0.7853982;

const double deg_10 = 10.0;
const double rad_10 = 0.1745329;

const double deg_20 = 20.0;
const double rad_20 = 0.3490659;

const double deg_30 = 30.0;
const double rad_30 = 0.5235988;

struct Rot3d {
  Rot3d() {
    // clang-format off
    roll45_pitch45_yaw45_mat << 0.5,        -0.1464466,  0.8535534, 
                                0.5,         0.8535534, -0.1464466, 
                                -0.7071068,  0.5,        0.5;

    roll10_pitch20_yaw30_mat << 0.8137977, -0.4409696,  0.3785223,
                                0.4698463,  0.8825641,  0.0180283,
                               -0.3420202,  0.1631759,  0.9254166;
    // clang-format on
    roll45_pitch45_yaw45_quat.x() = 0.1913417;
    roll45_pitch45_yaw45_quat.y() = 0.4619398;
    roll45_pitch45_yaw45_quat.z() = 0.1913417;
    roll45_pitch45_yaw45_quat.w() = 0.8446232;

    roll10_pitch20_yaw30_quat.x() = 0.0381346;
    roll10_pitch20_yaw30_quat.y() = 0.1893079;
    roll10_pitch20_yaw30_quat.z() = 0.2392983;
    roll10_pitch20_yaw30_quat.w() = 0.9515485;
  }
  Matrix3d roll45_pitch45_yaw45_mat;  // xyz轴都旋转45度
  Matrix3d roll10_pitch20_yaw30_mat;
  Quaterniond roll45_pitch45_yaw45_quat;
  Quaterniond roll10_pitch20_yaw30_quat;
};

Rot3d r;
