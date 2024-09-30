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

// tf::ToDeg
TEST(TestTransform3d, HandlesToDeg) {
  double toDeg = tf::ToDeg(rad_45);
  cout << "toDeg=" << toDeg << seg;
  double ret = abs(toDeg - deg_45);
  EXPECT_LE(ret, 1e-4) << "rad_45-deg_45=" << ret;
}

// tf::ToRad
TEST(TestTransform3d, HandlesToRad) {
  double toRad = tf::ToRad(deg_45);
  cout << "toRad=" << toRad << seg;
  double ret = abs(toRad - rad_45);
  EXPECT_LE(ret, 1e-4) << "deg_45-rad_45=" << ret;
}

// tf::EulerRad2Mat
TEST(TestTransform3d, HandlesEulerV3dRad2Mat) {
  auto V3d = Vector3d(rad_30, rad_20, rad_10);
  auto Mat = tf::EulerRad2Mat(V3d);
  Matrix3d ret = Mat - r.roll10_pitch20_yaw30_mat;
  cout << "v3d ret:\n" << ret << seg;
  for (int i = 0; i < ret.rows(); i++) {
    for (int j = 0; j < ret.cols(); j++) {
      ASSERT_LE(std::abs(ret(i, j)), 1e-4);
    }
  }
}

// tf::EulerDeg2Mat
TEST(TestTransform3d, HandlesEulerV3dDeg2Mat) {
  auto V3d = Vector3d(deg_30, deg_20, deg_10);
  auto Mat = tf::EulerDeg2Mat(V3d);
  Matrix3d ret = Mat - r.roll10_pitch20_yaw30_mat;
  cout << "v3d ret:\n" << ret << seg;
  for (int i = 0; i < ret.rows(); i++) {
    for (int j = 0; j < ret.cols(); j++) {
      ASSERT_LE(std::abs(ret(i, j)), 1e-4);
    }
  }
}

// tf::EulerDeg2Mat
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

// tf::EulerRad2Mat
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

// tf::EulerDeg2Quat
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

// tf::EulerRad2Quat
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

// tf::Mat2EulerRad
TEST(TestTransform3d, HandlesMat2EulerRadV3d) {
  Vector3d euler_rad = tf::Mat2EulerRad(r.roll10_pitch20_yaw30_mat);
  cout << "euler_rad=\n" << euler_rad << seg;
  ASSERT_LE(abs(euler_rad[0] - rad_30), 1e-4);
  ASSERT_LE(abs(euler_rad[1] - rad_20), 1e-4);
  ASSERT_LE(abs(euler_rad[2] - rad_10), 1e-4);
}

// tf::Mat2EulerDeg
TEST(TestTransform3d, HandlesMat2EulerDegV3d) {
  Vector3d euler_degs = tf::Mat2EulerDeg(r.roll10_pitch20_yaw30_mat);
  cout << "euler_degs=\n" << euler_degs << seg;
  ASSERT_LE(abs(euler_degs[0] - deg_30), 1e-4);
  ASSERT_LE(abs(euler_degs[1] - deg_20), 1e-4);
  ASSERT_LE(abs(euler_degs[2] - deg_10), 1e-4);
}

// tf::Mat2EulerDeg
TEST(TestTransform3d, HandlesMat2EulerDeg) {
  double roll_deg;
  double pitch_deg;
  double yaw_deg;
  tf::Mat2EulerDeg(r.roll10_pitch20_yaw30_mat, roll_deg, pitch_deg, yaw_deg);

  ASSERT_LE(abs(roll_deg - deg_10), 1e-4);
  ASSERT_LE(abs(pitch_deg - deg_20), 1e-4);
  ASSERT_LE(abs(yaw_deg - deg_30), 1e-4);
}

// tf::Mat2EulerRad
TEST(TestTransform3d, HandlesMat2EulerRad) {
  double roll_rad;
  double pitch_rad;
  double yaw_rad;
  tf::Mat2EulerRad(r.roll10_pitch20_yaw30_mat, roll_rad, pitch_rad, yaw_rad);

  ASSERT_LE(abs(roll_rad - rad_10), 1e-4);
  ASSERT_LE(abs(pitch_rad - rad_20), 1e-4);
  ASSERT_LE(abs(yaw_rad - rad_30), 1e-4);
}

// tf::Mat2Quat
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

// tf::Quat2EulerRad
TEST(TestTransform3d, HandlesQuat2EulerRadV3d) {
  Vector3d rad = tf::Quat2EulerRad(r.roll10_pitch20_yaw30_quat);
  cout << "rad=\n" << rad << seg;
  ASSERT_LE(abs(rad(0) - rad_30), 1e-4);
  ASSERT_LE(abs(rad(1) - rad_20), 1e-4);
  ASSERT_LE(abs(rad(2) - rad_10), 1e-4);
}

// tf::Quat2EulerDeg
TEST(TestTransform3d, HandlesQuat2EulerDegV3d) {
  Vector3d deg = tf::Quat2EulerDeg(r.roll10_pitch20_yaw30_quat);
  cout << "deg=\n" << deg << seg;
  ASSERT_LE(abs(deg(0) - deg_30), 1e-4);
  ASSERT_LE(abs(deg(1) - deg_20), 1e-4);
  ASSERT_LE(abs(deg(2) - deg_10), 1e-4);
}

// tf::Quat2EulerRad
TEST(TestTransform3d, HandlesQuat2EulerRad) {
  double roll_rad, pitch_rad, yaw_rad;
  tf::Quat2EulerRad(r.roll10_pitch20_yaw30_quat, roll_rad, pitch_rad, yaw_rad);
  cout << "roll_rad=" << roll_rad << seg;
  cout << "pitch_rad=" << pitch_rad << seg;
  cout << "yaw_rad=" << yaw_rad << seg;
  ASSERT_LE(abs(roll_rad - rad_10), 1e-4);
  ASSERT_LE(abs(pitch_rad - rad_20), 1e-4);
  ASSERT_LE(abs(yaw_rad - rad_30), 1e-4);
}

// tf::Quat2EulerDeg
TEST(TestTransform3d, HandlesQuat2EulerDeg) {
  double roll_deg, pitch_deg, yaw_deg;
  tf::Quat2EulerDeg(r.roll10_pitch20_yaw30_quat, roll_deg, pitch_deg, yaw_deg);
  cout << "roll_deg=" << roll_deg << seg;
  cout << "pitch_deg=" << pitch_deg << seg;
  cout << "yaw_deg=" << yaw_deg << seg;
  ASSERT_LE(abs(roll_deg - deg_10), 1e-4);
  ASSERT_LE(abs(pitch_deg - deg_20), 1e-4);
  ASSERT_LE(abs(yaw_deg - deg_30), 1e-4);
}

// tf::Quat2Mat
TEST(TestTransform3d, HandlesQuat2Mat) {
  Matrix3d rot_mat = tf::Quat2Mat(r.roll10_pitch20_yaw30_quat);
  Matrix3d ret = rot_mat - r.roll10_pitch20_yaw30_mat;
  cout << "10,20,30 rot_mat-roll10_pitch20_yaw30_mat= \n" << ret << seg;
  for (int i = 0; i < ret.rows(); i++) {
    for (int j = 0; j < ret.cols(); j++) {
      ASSERT_LE(std::abs(ret(i, j)), 1e-4);
    }
  }
}