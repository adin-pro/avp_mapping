/*
 * @Author: Ren Qian
 * @Date: 2020-02-28 19:13:26
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:34:38
 */

#ifndef _IMU_DATA_H_
#define _IMU_DATA_H_

#include <Eigen/Dense>
#include <cmath>
#include <deque>

namespace avp_mapping {
class IMUData {
public:
  struct LinearAcceleration {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct AngularVelocity {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  class Orientation {
  public:
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 0.0;

  public:
    void normlize() {
      double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
      x /= norm;
      y /= norm;
      z /= norm;
      w /= norm;
    }
  };

  double time = 0.0;
  LinearAcceleration linear_acceleration;
  AngularVelocity angular_velocity;
  Orientation orientation;

public:
  // 把四元数转换成旋转矩阵送出去
  Eigen::Matrix3d getOrientationMatrix();
  static bool syncData(std::deque<IMUData> &UnsyncedData,
                       std::deque<IMUData> &SyncedData, double sync_time);
  static bool ControlDuration(std::deque<IMUData> &pose_deque,
                              double duration);
};
} // namespace avp_mapping
#endif // _IMU_DATA_H_
