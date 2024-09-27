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

#include "Eigen/src/Geometry/Quaternion.h"
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

TEST(TestTransform3d, HandlesEulerDeg2Quat) {
  Quaterniond q = tf::EulerDeg2Quat(45, 45, 45);
  cout << "45,45,45 quat= \n" << q << seg;
  double det_q_x = abs(q.x() - r.roll45_pitch45_yaw45_quat.x());
  double det_q_y = abs(q.y() - r.roll45_pitch45_yaw45_quat.y());
  double det_q_z = abs(q.z() - r.roll45_pitch45_yaw45_quat.z());
  double det_q_w = abs(q.w() - r.roll45_pitch45_yaw45_quat.w());
  cout << "det_q_x=" << det_q_x << seg;
  cout << "det_q_y=" << det_q_y << seg;
  cout << "det_q_z=" << det_q_z << seg;
  cout << "det_q_w=" << det_q_w << seg;
  ASSERT_LE(det_q_x, 1e-4);
  ASSERT_LE(det_q_y, 1e-4);
  ASSERT_LE(det_q_z, 1e-4);
  ASSERT_LE(det_q_w, 1e-4);
}

TEST(TestTransform3d, HandlesEulerRad2Quat) {
  Quaterniond q =
      tf::EulerRad2Quat(tf::ToRad(45), tf::ToRad(45), tf::ToRad(45));
  cout << "45,45,45 quat= \n" << q << seg;
  double det_q_x = abs(q.x() - r.roll45_pitch45_yaw45_quat.x());
  double det_q_y = abs(q.y() - r.roll45_pitch45_yaw45_quat.y());
  double det_q_z = abs(q.z() - r.roll45_pitch45_yaw45_quat.z());
  double det_q_w = abs(q.w() - r.roll45_pitch45_yaw45_quat.w());
  cout << "det_q_x=" << det_q_x << seg;
  cout << "det_q_y=" << det_q_y << seg;
  cout << "det_q_z=" << det_q_z << seg;
  cout << "det_q_w=" << det_q_w << seg;
  ASSERT_LE(det_q_x, 1e-4);
  ASSERT_LE(det_q_y, 1e-4);
  ASSERT_LE(det_q_z, 1e-4);
  ASSERT_LE(det_q_w, 1e-4);
}

TEST(TestTransform3d, HandlesMat2EulerDeg) {
  double roll_deg;
  double pitch_deg;
  double yaw_deg;
  tf::Mat2EulerDeg(r.roll10_pitch20_yaw30_mat, roll_deg, pitch_deg, yaw_deg);

  ASSERT_LE(abs(roll_deg - deg_10), 1e-4);
  ASSERT_LE(abs(pitch_deg - deg_20), 1e-4);
  ASSERT_LE(abs(yaw_deg - deg_30), 1e-4);
}

TEST(TestTransform3d, HandlesMat2EulerRad) {
  double roll_rad;
  double pitch_rad;
  double yaw_rad;
  tf::Mat2EulerRad(r.roll10_pitch20_yaw30_mat, roll_rad, pitch_rad, yaw_rad);

  ASSERT_LE(abs(roll_rad - rad_10), 1e-4);
  ASSERT_LE(abs(pitch_rad - rad_20), 1e-4);
  ASSERT_LE(abs(yaw_rad - rad_30), 1e-4);
}

TEST(TestTransform3d, HandlesMat2Quat) {
  Quaterniond q = tf::Mat2Quat(r.roll10_pitch20_yaw30_mat);

  double det_q_x = abs(q.x() - r.roll10_pitch20_yaw30_quat.x());
  double det_q_y = abs(q.y() - r.roll10_pitch20_yaw30_quat.y());
  double det_q_z = abs(q.z() - r.roll10_pitch20_yaw30_quat.z());
  double det_q_w = abs(q.w() - r.roll10_pitch20_yaw30_quat.w());

  cout << "det_q_x=" << det_q_x << seg;
  cout << "det_q_y=" << det_q_y << seg;
  cout << "det_q_z=" << det_q_z << seg;
  cout << "det_q_w=" << det_q_w << seg;

  ASSERT_LE(det_q_x, 1e-4);
  ASSERT_LE(det_q_y, 1e-4);
  ASSERT_LE(det_q_z, 1e-4);
  ASSERT_LE(det_q_w, 1e-4);
}