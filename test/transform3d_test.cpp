/**
 * @file transform3d_test.cpp
 * @author huizeyu (huizeyu@siasun.com)
 * @brief
 * @version 0.1
 * @date 2024-09-26
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "transform3d.h"

#include <gtest/gtest.h>

#include "share_data_test.hpp"
TEST(TestTransform3d, HandlesToDeg) {
  double toDeg = tf::ToDeg(rad_45);
  cout << "toDeg=" << toDeg << seg;
  double ret = abs(toDeg - deg_45);
  EXPECT_LE(ret, 1e-4) << "rad_45-deg_45=" << ret;
}

TEST(TestTransform3d, HandlesToRad) {
  double toRad = tf::ToRad(deg_45);
  cout << "toRad=" << toRad << seg;
  double ret = abs(toRad - rad_45);
  EXPECT_LE(ret, 1e-4) << "deg_45-rad_45=" << ret;
}

TEST(TestTransform3d, HandlesEulerDeg2Mat) {
  Rot3d r;
  Matrix3d mat = tf::EulerDeg2Mat(45, 45, 45);
  cout << "45,45,45 mat= \n" << mat << seg;
  Matrix3d ret = mat - r.roll45_pitch45_yaw45_mat;
  cout << "45,45,45 mat-roll45_pitch45_yaw45_mat= \n" << ret << seg;
  for (int i = 0; i < ret.rows(); i++) {
    for (int j = 0; j < ret.cols(); j++) {
      ASSERT_LE(std::abs(ret(i, j)), 1e-4);
    }
  }
}

TEST(TestTransform3d, HandlesEulerRad2Mat) {
  Rot3d r;
  Matrix3d rad_mat =
      tf::EulerRad2Mat(tf::ToRad(45), tf::ToRad(45), tf::ToRad(45));
  cout << "45,45,45 rad_mat= \n" << rad_mat << seg;
  Matrix3d ret = rad_mat - r.roll45_pitch45_yaw45_mat;
  cout << "45,45,45 rad_mat-roll45_pitch45_yaw45_mat= \n" << ret << seg;
  for (int i = 0; i < ret.rows(); i++) {
    for (int j = 0; j < ret.cols(); j++) {
      ASSERT_LE(std::abs(ret(i, j)), 1e-4);
    }
  }
}