/*
 * @Author: Ren Qian
 * @Date: 2020-02-27 23:10:56
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-12 21:05:23
 */

#ifndef _POSE_DATA_H_
#define _POSE_DATA_H_

#include <Eigen/Dense>
#include <deque>

namespace avp_mapping {
class PoseData {
public:
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  double time = 0.0;

public:
  Eigen::Quaterniond getQuaternion();
  static bool syncData(std::deque<PoseData> &unsyncedData,
                       std::deque<PoseData> &syncedData, double sync_time);
  static bool syncDataWithoutInterpolation(std::deque<PoseData> &unsyncedData,
                                           double sync_time);
  static bool controlDuration(std::deque<PoseData> &pose_deque,
                              double duration);
  static bool getPoseDataByTS(std::deque<PoseData> &pose_deque,
                               double timestamp, PoseData &result);
  static bool isFarEnough(const PoseData& last_pose, const PoseData& curr_pose, double thre);

  void printPos();
};
} // namespace avp_mapping

#endif // _POSE_DATA_H_
