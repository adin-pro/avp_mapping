/*
 * @Author: Ren Qian
 * @Date: 2020-02-28 18:50:16
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-03 21:21:54
 */

#include "sensor_data/pose_data.hpp"

#include <deque>

namespace avp_mapping {

Eigen::Quaternionf PoseData::getQuaternion() {
  Eigen::Quaternionf q;
  q = pose.block<3, 3>(0, 0);
  return q;
}

bool PoseData::syncData(std::deque<PoseData> &unsyncedData,
                        std::deque<PoseData> &syncedData, double sync_time) {
  while (unsyncedData.size() >= 2) {
    if (unsyncedData.front().time > sync_time)
      return false;
    if (unsyncedData.at(1).time < sync_time) {
      unsyncedData.pop_front();
      continue;
    }
    if (sync_time - unsyncedData.front().time > 0.2) {
      unsyncedData.pop_front();
      break;
    }
    if (unsyncedData.at(1).time - sync_time > 0.2) {
      unsyncedData.pop_front();
      break;
    }
    break;
  }
  if (unsyncedData.size() < 2)
    return false;

  PoseData front_data = unsyncedData.at(0);
  PoseData back_data = unsyncedData.at(1);
  PoseData synced_data;

  double front_scale =
      (back_data.time - sync_time) / (back_data.time - front_data.time);
  double back_scale =
      (sync_time - front_data.time) / (back_data.time - front_data.time);
  synced_data.time = sync_time;

  synced_data.pose(0, 3) =
      front_data.pose(0, 3) * front_scale + back_data.pose(0, 3) * back_scale;
  synced_data.pose(1, 3) =
      front_data.pose(1, 3) * front_scale + back_data.pose(1, 3) * back_scale;
  synced_data.pose(2, 3) =
      front_data.pose(2, 3) * front_scale + back_data.pose(2, 3) * back_scale;
  Eigen::Quaternionf qf = front_data.getQuaternion();
  Eigen::Quaternionf qb = back_data.getQuaternion();
  Eigen::Quaternionf q;
  q.x() = qf.x() * front_scale + qb.x() * back_scale;
  q.y() = qf.y() * front_scale + qb.y() * back_scale;
  q.z() = qf.z() * front_scale + qb.z() * back_scale;
  q.w() = qf.w() * front_scale + qb.w() * back_scale;
  synced_data.pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  syncedData.push_back(synced_data);
  return true;
}

bool PoseData::syncDataWithoutInterpolation(std::deque<PoseData> &unsyncedData,
                                            double sync_time) {
  while (unsyncedData.size() > 0) {
    PoseData pose_data = unsyncedData.front();
    if (pose_data.time < sync_time || pose_data.time - sync_time > 0.015) {
      unsyncedData.pop_front();
    } else {
      return true;
    }
    
  }                                            
  return false;
}

} // namespace avp_mapping
