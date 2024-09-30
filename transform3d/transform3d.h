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

// 旋转矩阵转欧拉角 角度 弧度===============O
// 旋转矩阵转四元数 ======================O

// 四元数转欧拉角  角度 弧度===============O
// 四元数转旋转矩阵=======================O

// 齐次矩阵相关

#ifndef __TRANSFORM3D_H__
#define __TRANSFORM3D_H__

#include <Eigen/Dense>
#include <cmath>

#include "Eigen/src/Core/Matrix.h"
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
   * @brief 弧度表示的欧拉角转换为旋转矩阵，按ZYX的顺序依次旋转
   *
   * @param euler_rad 弧度表示 yaw:v[0] pitch:v[1] roll[2]
   * @return Matrix3d 旋转矩阵
   */
  static inline Matrix3d EulerRad2Mat(Vector3d euler_rad) {
    AngleAxisd rollAngle(AngleAxisd(euler_rad[2], Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(euler_rad[1], Vector3d::UnitY()));
    AngleAxisd yawAngle(AngleAxisd(euler_rad[0], Vector3d::UnitZ()));
    Matrix3d rotation_matrix;
    rotation_matrix = yawAngle * pitchAngle * rollAngle;
    return rotation_matrix;
  }

  /**
   * @brief 角度表示的欧拉角转换为旋转矩阵，按ZYX的顺序依次旋转
   *
   * @param euler_deg 角度表示 yaw:v[0] pitch:v[1] roll[2]
   * @return Matrix3d 旋转矩阵
   */
  static inline Matrix3d EulerDeg2Mat(Vector3d euler_deg) {
    euler_deg = euler_deg * M_PI / 180.0;
    AngleAxisd rollAngle(AngleAxisd(euler_deg[2], Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(euler_deg[1], Vector3d::UnitY()));
    AngleAxisd yawAngle(AngleAxisd(euler_deg[0], Vector3d::UnitZ()));
    Matrix3d rotation_matrix;
    rotation_matrix = yawAngle * pitchAngle * rollAngle;
    return rotation_matrix;
  }

  /**
   * @brief 角度表示的欧拉角转换为旋转矩阵，按ZYX的顺序依次旋转
   *
   * @param roll_deg 角度表示的roll   绕x旋转
   * @param pitch_deg 角度表示的pitch 绕y旋转
   * @param yaw_deg 角度表示的yaw     绕z旋转
   * @return Matrix3d 旋转矩阵
   */
  static inline Matrix3d EulerDeg2Mat(double roll_deg, double pitch_deg,
                                      double yaw_deg) {
    Vector3d euler_deg{yaw_deg, pitch_deg, roll_deg};
    return EulerDeg2Mat(euler_deg);
  }

  /**
   * @brief 弧度制欧拉角转旋转矩阵，按ZYX的顺序依次旋转
   *
   * @param roll_rad  弧度表示的roll  绕x旋转
   * @param pitch_rad 弧度表示的pitch 绕y旋转
   * @param yaw_rad   弧度表示的yaw   绕z旋转
   * @return Matrix3d 旋转矩阵
   */
  static inline Matrix3d EulerRad2Mat(double roll_rad, double pitch_rad,
                                      double yaw_rad) {
    Vector3d euler_rad{yaw_rad, pitch_rad, roll_rad};
    return EulerRad2Mat(euler_rad);
  }

  /**
   * @brief 角度制欧拉角转四元数 按ZYX的顺序依次旋转
   *
   * @param roll_deg 角度表示的roll   绕x旋转
   * @param pitch_deg 角度表示的pitch 绕y旋转
   * @param yaw_deg 角度表示的yaw     绕z旋转
   * @return Quaterniond 四元数
   */
  static inline Quaterniond EulerDeg2Quat(double roll_deg, double pitch_deg,
                                          double yaw_deg) {
    return Eigen::AngleAxisd(ToRad(yaw_deg), ::Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(ToRad(pitch_deg), ::Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(ToRad(roll_deg), ::Eigen::Vector3d::UnitX());
  }

  /**
   * @brief 弧度制欧拉角转四元数 按ZYX的顺序依次旋转
   *
   * @param roll_rad  弧度表示的roll  绕x旋转
   * @param pitch_rad 弧度表示的pitch 绕y旋转
   * @param yaw_rad   弧度表示的yaw   绕z旋转
   * @return Quaterniond
   */
  static inline Quaterniond EulerRad2Quat(double roll_rad, double pitch_rad,
                                          double yaw_rad) {
    return Eigen::AngleAxisd(yaw_rad, ::Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(pitch_rad, ::Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(roll_rad, ::Eigen::Vector3d::UnitX());
  }

  /*===============欧拉角转换================*/

  /*===============旋转矩阵转换================*/

  /**
   * @brief 旋转矩阵转弧度制欧拉角 按ZYX的顺序依次旋转
   *
   * @param rot_mat 旋转矩阵
   * @return Vector3d yaw:v[0] pitch:v[1] roll[2]
   */
  static inline Vector3d Mat2EulerRad(Matrix3d rot_mat) {
    return rot_mat.eulerAngles(2, 1, 0);
  }

  /**
   * @brief 旋转矩阵转角度制欧拉角 按ZYX的顺序依次旋转
   *
   * @param rot_mat 旋转矩阵
   * @return Vector3d yaw:v[0] pitch:v[1] roll[2]
   */
  static inline Vector3d Mat2EulerDeg(Matrix3d rot_mat) {
    return rot_mat.eulerAngles(2, 1, 0) * 180 / M_PI;
  }

  /**
   * @brief 旋转矩阵转角度制欧拉角 按ZYX的顺序依次旋转
   *
   * @param rot_mat   旋转矩阵
   * @param roll_deg  出参 角度表示的roll
   * @param pitch_deg 出参 角度表示的pitch
   * @param yaw_deg   出参 角度表示的yaw
   */
  static inline void Mat2EulerDeg(Matrix3d rot_mat, double &roll_deg,
                                  double &pitch_deg, double &yaw_deg) {
    Vector3d Euler = rot_mat.eulerAngles(2, 1, 0);
    Euler = Euler * 180 / M_PI;
    roll_deg = Euler(2);
    pitch_deg = Euler(1);
    yaw_deg = Euler(0);
  }

  /**
   * @brief 旋转矩阵转弧度制欧拉角 按ZYX的顺序依次旋转
   *
   * @param rot_mat   旋转矩阵
   * @param roll_rad  出参 弧度表示的roll
   * @param pitch_rad 出参 弧度表示的pitch
   * @param yaw_rad   出参 弧度表示的yaw
   */
  static inline void Mat2EulerRad(Matrix3d rot_mat, double &roll_rad,
                                  double &pitch_rad, double &yaw_rad) {
    Vector3d Euler = rot_mat.eulerAngles(2, 1, 0);
    roll_rad = Euler(2);
    pitch_rad = Euler(1);
    yaw_rad = Euler(0);
  }

  static inline Quaterniond Mat2Quat(Matrix3d rot_mat) {
    return Quaterniond(rot_mat);
  }

  /*===============旋转矩阵转换================*/

  /*===============四元数转换================*/

  /**
   * @brief 四元数转弧度制欧拉角 按ZYX的顺序依次旋转
   *
   * @param quat 待转换的四元数
   * @return Vector3d 弧度表示 yaw:v[0] pitch:v[1] roll[2]
   */
  static inline Vector3d Quat2EulerRad(Quaterniond quat) {
    return quat.toRotationMatrix().eulerAngles(2, 1, 0);
  }

  /**
   * @brief 四元数转角度制欧拉角 按ZYX的顺序依次旋转
   *
   * @param quat
   * @return Vector3d 角度表示 yaw:v[0] pitch:v[1] roll[2]
   */
  static inline Vector3d Quat2EulerDeg(Quaterniond quat) {
    return Quat2EulerRad(quat) * 180 / M_PI;
  }

  /**
   * @brief 四元数转弧度制欧拉角 按ZYX的顺序依次旋转
   *
   * @param quat       待转换的四元数
   * @param roll_rad   出参 弧度表示的roll
   * @param pitch_rad  出参 弧度表示的pitch
   * @param yaw_rad    出参 弧度表示的yaw
   */
  static inline void Quat2EulerRad(Quaterniond quat, double &roll_rad,
                                   double &pitch_rad, double &yaw_rad) {
    Vector3d Euler = Quat2EulerRad(quat);
    roll_rad = Euler(2);
    pitch_rad = Euler(1);
    yaw_rad = Euler(0);
  }

  /**
   * @brief 四元数转角度制欧拉角 按ZYX的顺序依次旋转
   *
   * @param quat      待转换的四元数
   * @param roll_deg  出参 角度表示的roll
   * @param pitch_deg 出参 角度表示的pitch
   * @param yaw_deg   出参 角度表示的yaw
   */
  static inline void Quat2EulerDeg(Quaterniond quat, double &roll_deg,
                                   double &pitch_deg, double &yaw_deg) {
    Vector3d Euler = Quat2EulerDeg(quat);
    roll_deg = Euler(2);
    pitch_deg = Euler(1);
    yaw_deg = Euler(0);
  }

  /**
   * @brief 四元数转旋转矩阵
   *
   * @param quat 待转换的四元数
   * @return Matrix3d 旋转矩阵
   */
  static inline Matrix3d Quat2Mat(Quaterniond quat) {
    return quat.toRotationMatrix();
  }

  /*===============四元数转换================*/

 private:
 public:
 private:
};
}  // namespace ToolComponent

using tf = ToolComponent::Transform3D;

#endif