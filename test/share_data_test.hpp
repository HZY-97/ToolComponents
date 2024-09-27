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

struct Rot3d {
  Rot3d() {
    // clang-format off
    roll45_pitch45_yaw45_mat <<  0.5,       -0.1464466,  0.8535534, 
                                 0.5,        0.8535534, -0.1464466, 
                                -0.7071068,  0.5,        0.5;
    // clang-format on
  }
  Matrix3d roll45_pitch45_yaw45_mat;  // xyz轴都旋转45度
};
