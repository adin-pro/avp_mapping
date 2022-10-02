/*
 * @Description: velocity 数据
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:27:40
 */

/*
 * @Author: ding.yin
 * @Date: 2022-10-02 20:20:35
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-02 20:21:09
 */

#ifndef _VELOCITY_DATA_H_
#define _VELOCITY_DATA_H_

#include <Eigen/Dense>
#include <deque>

namespace avp_mapping {
class VelocityData {
public:
  struct LinearVelocity {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct AngularVelocity {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  double time = 0.0;
  LinearVelocity linear_velocity;
  AngularVelocity angular_velocity;

public:
  static bool syncData(std::deque<VelocityData> &UnsyncedData,
                       std::deque<VelocityData> &SyncedData, double sync_time);
  void transformCoordinate(Eigen::Matrix4f transform_matrix);
};
} // namespace avp_mapping

#endif // _VELOCITY_DATA_H_
