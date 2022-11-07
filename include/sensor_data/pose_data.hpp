/*
 * @Author: Ren Qian
 * @Date: 2020-02-27 23:10:56
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-03 21:23:36
 */

#ifndef _POSE_DATA_H_
#define _POSE_DATA_H_

#include <Eigen/Dense>
#include <deque>

namespace avp_mapping {
class PoseData {
public:
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  double time = 0.0;

public:
  Eigen::Quaternionf getQuaternion();
  static bool syncData(std::deque<PoseData> &unsyncedData,
                       std::deque<PoseData> &syncedData, double sync_time);
  static bool syncDataWithoutInterpolation(std::deque<PoseData> &unsyncedData,
                                           double sync_time);
};
} // namespace avp_mapping

#endif // _POSE_DATA_H_
