/**
 * @file transform3d.h
 * @author huizeyu (huizeyu@siasun.com)
 * @brief
 * @version 0.1
 * @date 2024-09-26
 *
 * @copyright Copyright (c) 2024
 *
 */

// 弧度转角度============================O
// 角度转弧度============================O

// 欧拉角转旋转矩阵 角度 弧度===============O
// 欧拉角转四元数  角度 弧度================O

// 旋转矩阵转欧拉角 角度 弧度
// 旋转矩阵转四元数

// 四元数转欧拉角  角度 弧度
// 四元数转旋转矩阵

// 齐次矩阵相关

#ifndef __TRANSFORM3D_H__
#define __TRANSFORM3D_H__

#include <Eigen/Dense>
#include <cmath>

#include "Eigen/src/Geometry/Quaternion.h"
#include "Eigen/src/Geometry/Transform.h"

using namespace Eigen;

namespace ToolComponent {
class Transform3D {
 public:
  Transform3D() {
  }
  virtual ~Transform3D() {
  }

  /*===============角度弧度转换================*/

  /**
   * @brief 弧度转角度
   *
   * @param rad 弧度
   * @return double 角度
   */
  static inline double ToDeg(double rad) {
    return rad * 180.0 / M_PI;
  }

  /**
   * @brief 角度转弧度
   *
   * @param deg 角度
   * @return double 弧度
   */
  static inline double ToRad(double deg) {
    return deg * M_PI / 180.0;
  }

  /*===============角度弧度转换================*/

  /*===============欧拉角转换================*/

  /**
   * @brief 角度表示的欧拉角转换为旋转矩阵，按XYZ的顺序依次旋转
   *
   * @param roll_deg 角度表示的roll   绕x旋转
   * @param pitch_deg 角度表示的pitch 绕y旋转
   * @param yaw_deg 角度表示的yaw     绕z旋转
   * @return Matrix3d 旋转矩阵
   */
  static inline Matrix3d EulerDeg2Mat(double roll_deg, double pitch_deg,
                                      double yaw_deg) {
    roll_deg = ToRad(roll_deg);
    pitch_deg = ToRad(pitch_deg);
    yaw_deg = ToRad(yaw_deg);
    AngleAxisd rollAngle(AngleAxisd(roll_deg, Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(pitch_deg, Vector3d::UnitY()));
    AngleAxisd yawAngle(AngleAxisd(yaw_deg, Vector3d::UnitZ()));
    Matrix3d rotation_matrix;
    rotation_matrix = rollAngle * pitchAngle * yawAngle;
    return rotation_matrix;
  }

  /**
   * @brief 弧度制欧拉角转旋转矩阵，按XYZ的顺序依次旋转
   *
   * @param roll_rad  弧度表示的roll  绕x旋转
   * @param pitch_rad 弧度表示的pitch 绕y旋转
   * @param yaw_rad   弧度表示的yaw   绕z旋转
   * @return Matrix3d 旋转矩阵
   */
  static inline Matrix3d EulerRad2Mat(double roll_rad, double pitch_rad,
                                      double yaw_rad) {
    AngleAxisd rollAngle(AngleAxisd(roll_rad, Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(pitch_rad, Vector3d::UnitY()));
    AngleAxisd yawAngle(AngleAxisd(yaw_rad, Vector3d::UnitZ()));
    Matrix3d rotation_matrix;
    rotation_matrix = rollAngle * pitchAngle * yawAngle;
    return rotation_matrix;
  }

  /**
   * @brief 角度制欧拉角转四元数 按XYZ的顺序依次旋转
   *
   * @param roll_deg 角度表示的roll   绕x旋转
   * @param pitch_deg 角度表示的pitch 绕y旋转
   * @param yaw_deg 角度表示的yaw     绕z旋转
   * @return Quaterniond 四元数
   */
  static inline Quaterniond EulerDeg2Quat(double roll_deg, double pitch_deg,
                                          double yaw_deg) {
    roll_deg = ToRad(roll_deg);
    pitch_deg = ToRad(pitch_deg);
    yaw_deg = ToRad(yaw_deg);
    return Eigen::AngleAxisd(roll_deg, ::Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(pitch_deg, ::Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(yaw_deg, ::Eigen::Vector3d::UnitZ());
  }

  /**
   * @brief 弧度制欧拉角转四元数 按XYZ的顺序依次旋转
   *
   * @param roll_rad  弧度表示的roll  绕x旋转
   * @param pitch_rad 弧度表示的pitch 绕y旋转
   * @param yaw_rad   弧度表示的yaw   绕z旋转
   * @return Quaterniond
   */
  static inline Quaterniond EulerRad2Quat(double roll_rad, double pitch_rad,
                                          double yaw_rad) {
    return Eigen::AngleAxisd(roll_rad, ::Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(pitch_rad, ::Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(yaw_rad, ::Eigen::Vector3d::UnitZ());
  }

 private:
 public:
 private:
};
}  // namespace ToolComponent

using tf = ToolComponent::Transform3D;

#endif